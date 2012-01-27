
#include <errno.h>
#include <pthread.h>
#include <termios.h>
#include <fcntl.h>
#include <sys/epoll.h>
#include <math.h>
#include <time.h>
#include <sys/time.h>
#include <semaphore.h>
#include <signal.h>
#include <unistd.h>

#define  LOG_TAG  "gps_vimm"

#include <cutils/log.h>
#include <cutils/sockets.h>
#include <cutils/properties.h>
#include <hardware_legacy/gps.h>

#define  GPS_DEBUG  0

#define  DFR(...)   LOGD(__VA_ARGS__)

#if GPS_DEBUG
#define  D(...)   LOGE(__VA_ARGS__)
#else
#define  D(...)   ((void)0)
#endif

#define GPS_STATUS_CB(_cb, _s)    \
    if ((_cb).status_cb) {          \
    GpsStatus gps_status;         \
    gps_status.status = (_s);     \
    (_cb).status_cb(&gps_status); \
    DFR("gps status callback: 0x%x", _s); \
    }

/* Nmea Parser stuff */
#define  NMEA_MAX_SIZE  83

enum {
    STATE_QUIT  = 0,
    STATE_INIT  = 1,
    STATE_START = 2
};

typedef struct {
    int     pos;
    int     overflow;
    int     utc_year;
    int     utc_mon;
    int     utc_day;
    int     utc_diff;
    GpsLocation  fix;
    GpsSvStatus  sv_status;
    int     sv_status_changed;
    char    in[ NMEA_MAX_SIZE+1 ];
} NmeaReader;

/* Since NMEA parser requires lcoks */
#define GPS_STATE_LOCK_FIX(_s)         \
{                                      \
    int ret;                             \
    do {                                 \
        ret = sem_wait(&(_s)->fix_sem);    \
    } while (ret < 0 && errno == EINTR);   \
}

#define GPS_STATE_UNLOCK_FIX(_s)       \
    sem_post(&(_s)->fix_sem)

typedef struct {
    int                     init;
    int                     fd;
    GpsCallbacks            callbacks;
    pthread_t               thread;
    pthread_t               tmr_thread;
    int                     control[2];
    int                     fix_freq;
    sem_t                   fix_sem;
    int                     first_fix;
    NmeaReader              reader;

} GpsState;

static GpsState  _gps_state[1];
static GpsState *gps_state = _gps_state;

//#define GPS_POWER_IF "/sys/bus/platform/devices/neo1973-pm-gps.0/power_on"

#define GPS_DEV_SLOW_UPDATE_RATE (10)
#define GPS_DEV_HIGH_UPDATE_RATE (1)

#define GPS_DEV_LOW_BAUD  (B9600)
#define GPS_DEV_HIGH_BAUD (B19200)

static void gps_dev_init(int fd);
static void gps_dev_deinit(int fd);
static void gps_dev_start(int fd);
static void gps_dev_stop(int fd);
static void *gps_timer_thread( void*  arg );

/*****************************************************************/
/*****************************************************************/
/*****                                                       *****/
/*****       N M E A   T O K E N I Z E R    *****/
/*****                                                       *****/
/*****************************************************************/
/*****************************************************************/

typedef struct {
    const char*  p;
    const char*  end;
} Token;

#define  MAX_NMEA_TOKENS  32

typedef struct {
    int     count;
    Token   tokens[ MAX_NMEA_TOKENS ];
} NmeaTokenizer;

static int nmea_tokenizer_init( NmeaTokenizer*  t, const char*  p, const char*  end )
{
    int    count = 0;
    char*  q;
// the initial '$' is optional
    if (p < end && p[0] == '$')
        p += 1;
// remove trailing newline
    if (end > p && end[-1] == '\n') 
    {
        end -= 1;
        if (end > p && end[-1] == '\r')
            end -= 1;
    }
// get rid of checksum at the end of the sentecne
    if (end >= p+3 && end[-3] == '*') 
    {
        end -= 3;
    }
    while (p < end) 
    {
        const char*  q = p;
        q = memchr(p, ',', end-p);
        if (q == NULL)
            q = end;
        if (count < MAX_NMEA_TOKENS) 
        {
            t->tokens[count].p   = p;
            t->tokens[count].end = q;
            count += 1;
        }
        if (q < end)
           q += 1;
        p = q;
    }
    t->count = count;
    return count;
}

static Token nmea_tokenizer_get( NmeaTokenizer*  t, int  index )
{
    Token  tok;
    static const char*  dummy = "";
    if (index < 0 || index >= t->count)
    {
        tok.p = tok.end = dummy;
    } else
        tok = t->tokens[index];
    return tok;
}


static int str2int( const char*  p, const char*  end )
{
    int   result = 0;
    int   len    = end - p;

    if (len == 0) 
    {
        return -1;
    }
    for ( ; len > 0; len--, p++ )
    {
        int  c;
        if (p >= end)
            goto Fail;
        c = *p - '0';
        if ((unsigned)c >= 10)
            goto Fail;
        result = result*10 + c;
    }
    return  result;

Fail:
    return -1;
}

static double str2float( const char*  p, const char*  end )
{
    int   result = 0;
    int   len    = end - p;
    char  temp[16];

    if (len == 0) 
    {
        return -1.0;
    }
    if (len >= (int)sizeof(temp))
        return 0.;
    memcpy( temp, p, len );
    temp[len] = 0;
    return strtod( temp, NULL );
}

/*****************************************************************/
/*****************************************************************/
/*****                                                       *****/
/*****       N M E A   P A R S E R           *****/
/*****                                                       *****/
/*****************************************************************/
/*****************************************************************/

static void nmea_reader_update_utc_diff( NmeaReader*  r )
{
    time_t         now = time(NULL);
    struct tm      tm_local;
    struct tm      tm_utc;
    long           time_local, time_utc;
    gmtime_r( &now, &tm_utc );
    localtime_r( &now, &tm_local );
    time_local = tm_local.tm_sec +
                 60*(tm_local.tm_min +
                 60*(tm_local.tm_hour +
                 24*(tm_local.tm_yday +
                 365*tm_local.tm_year)));
    time_utc = tm_utc.tm_sec +
               60*(tm_utc.tm_min +
               60*(tm_utc.tm_hour +
               24*(tm_utc.tm_yday +
               365*tm_utc.tm_year)));
    r->utc_diff = time_utc - time_local;
}


static void nmea_reader_init( NmeaReader*  r )
{
    memset( r, 0, sizeof(*r) );
    r->pos      = 0;
    r->overflow = 0;
    r->utc_year = -1;
    r->utc_mon  = -1;
    r->utc_day  = -1;
    nmea_reader_update_utc_diff( r );
}

static int nmea_reader_update_time( NmeaReader*  r, Token  tok )
{
    int        hour, minute;
    double     seconds;
    struct tm  tm;
    time_t     fix_time;
    if (tok.p + 6 > tok.end)
        return -1;
    if (r->utc_year < 0) 
    {
// no date yet, get current one
        time_t  now = time(NULL);
        gmtime_r( &now, &tm );
        r->utc_year = tm.tm_year + 1900;
        r->utc_mon  = tm.tm_mon + 1;
        r->utc_day  = tm.tm_mday;
    }
    hour    = str2int(tok.p,   tok.p+2);
    minute  = str2int(tok.p+2, tok.p+4);
    seconds = str2float(tok.p+4, tok.end);
    tm.tm_hour = hour;
    tm.tm_min  = minute;
    tm.tm_sec  = (int) seconds;
    tm.tm_year = r->utc_year - 1900;
    tm.tm_mon  = r->utc_mon - 1;
    tm.tm_mday = r->utc_day;
    fix_time = mktime( &tm ) + r->utc_diff;
    r->fix.timestamp = (long long)fix_time * 1000;
    return 0;
}

static int nmea_reader_update_cdate( NmeaReader*  r, Token  tok_d, Token tok_m, Token tok_y )
{
    if ( (tok_d.p + 2 > tok_d.end) ||
       (tok_m.p + 2 > tok_m.end) ||
       (tok_y.p + 4 > tok_y.end) )
        return -1;
    r->utc_day = str2int(tok_d.p,   tok_d.p+2);
    r->utc_mon = str2int(tok_m.p, tok_m.p+2);
    r->utc_year = str2int(tok_y.p, tok_y.end+4);
    return 0;
}

static int nmea_reader_update_date( NmeaReader*  r, Token  date, Token  time )
{
    Token  tok = date;
    int    day, mon, year;
    if (tok.p + 6 != tok.end) 
    {
        D("date not properly formatted: '%.*s'", tok.end-tok.p, tok.p);
        return -1;
    }
    day  = str2int(tok.p, tok.p+2);
    mon  = str2int(tok.p+2, tok.p+4);
    year = str2int(tok.p+4, tok.p+6) + 2000;
    if ((day|mon|year) < 0) 
    {
        D("date not properly formatted: '%.*s'", tok.end-tok.p, tok.p);
        return -1;
    }
    r->utc_year  = year;
    r->utc_mon   = mon;
    r->utc_day   = day;
    return nmea_reader_update_time( r, time );
}


static double convert_from_hhmm( Token  tok )
{
    double  val     = str2float(tok.p, tok.end);
    int     degrees = (int)(floor(val) / 100);
    double  minutes = val - degrees*100.;
    double  dcoord  = degrees + minutes / 60.0;
    return dcoord;
}


static int nmea_reader_update_latlong( NmeaReader*  r,
                            Token        latitude,
                            char         latitudeHemi,
                            Token        longitude,
                            char         longitudeHemi )
{
    double   lat, lon;
    Token    tok;
    tok = latitude;
    if (tok.p + 6 > tok.end)
    {
        D("latitude is too short: '%.*s'", tok.end-tok.p, tok.p);
        return -1;
    }
    lat = convert_from_hhmm(tok);
    if (latitudeHemi == 'S')
        lat = -lat;
    tok = longitude;
    if (tok.p + 6 > tok.end)
    {
        D("longitude is too short: '%.*s'", tok.end-tok.p, tok.p);
        return -1;
    }
    lon = convert_from_hhmm(tok);
    if (longitudeHemi == 'W')
        lon = -lon;

    r->fix.flags    |= GPS_LOCATION_HAS_LAT_LONG;
    r->fix.latitude  = lat;
    r->fix.longitude = lon;
    return 0;
}


static int nmea_reader_update_altitude( NmeaReader*  r,
                             Token        altitude,
                             Token        units )
{
    double  alt;
    Token   tok = altitude;
   if (tok.p >= tok.end)
       return -1;
   r->fix.flags   |= GPS_LOCATION_HAS_ALTITUDE;
   r->fix.altitude = str2float(tok.p, tok.end);
   return 0;
}

static int nmea_reader_update_accuracy( NmeaReader*  r,
                             Token        accuracy )
{
    double  acc;
    Token   tok = accuracy;
    if (tok.p >= tok.end)
        return -1;
    r->fix.accuracy = str2float(tok.p, tok.end);
    if (r->fix.accuracy == 99.99)
    {
        return 0;
    }
    r->fix.flags |= GPS_LOCATION_HAS_ACCURACY;
    return 0;
}

static int nmea_reader_update_bearing( NmeaReader*  r,
                            Token        bearing )
{
    double  alt;
    Token   tok = bearing;
    if (tok.p >= tok.end)
        return -1;
    r->fix.flags   |= GPS_LOCATION_HAS_BEARING;
    r->fix.bearing  = str2float(tok.p, tok.end);
    return 0;
}


static int nmea_reader_update_speed( NmeaReader*  r,
                          Token        speed )
{
    double  alt;
    Token   tok = speed;

    if (tok.p >= tok.end)
        return -1;
    r->fix.flags   |= GPS_LOCATION_HAS_SPEED;
    r->fix.speed    = str2float(tok.p, tok.end) * 0.514444;   // change to m/s
    return 0;
}


static void nmea_reader_parse( NmeaReader*  r )
{
	D("nmea_reader_parse IN");
/* we received a complete sentence, now parse it to generate
    * a new GPS fix...
 */
    NmeaTokenizer  tzer[1];
    Token          tok;
    D("Received: '%.*s'", r->pos, r->in);
    if (r->pos < 9)
    {
        D("Too short. discarded.");
        return;
    }
    nmea_tokenizer_init(tzer, r->in, r->in + r->pos);
#if GPS_DEBUG
    {
        int  n;
        D("Found %d tokens", tzer->count);
        for (n = 0; n < tzer->count; n++)
        {
            Token  tok = nmea_tokenizer_get(tzer,n);
            D("%2d: '%.*s'", n, tok.end-tok.p, tok.p);
        }
    }
#endif
    tok = nmea_tokenizer_get(tzer, 0);
    if (tok.p + 5 > tok.end)
    {
        D("sentence id '%.*s' too short, ignored.", tok.end-tok.p, tok.p);
        return;
    }
// ignore first two characters.
    tok.p += 2;
    if ( !memcmp(tok.p, "GGA", 3) ) 
    {
// GPS fix
        D("GGA parser IN");
        Token  tok_fixstaus      = nmea_tokenizer_get(tzer,6);
        if (tok_fixstaus.p[0] > '0') 
        {
            Token  tok_time          = nmea_tokenizer_get(tzer,1);
            Token  tok_latitude      = nmea_tokenizer_get(tzer,2);
            Token  tok_latitudeHemi  = nmea_tokenizer_get(tzer,3);
            Token  tok_longitude     = nmea_tokenizer_get(tzer,4);
            Token  tok_longitudeHemi = nmea_tokenizer_get(tzer,5);
            Token  tok_usedInfix = nmea_tokenizer_get(tzer,7);
            Token  tok_altitude      = nmea_tokenizer_get(tzer,9);
            Token  tok_altitudeUnits = nmea_tokenizer_get(tzer,10);
            nmea_reader_update_time(r, tok_time);
            nmea_reader_update_latlong(r, tok_latitude,
                                      tok_latitudeHemi.p[0],
                                      tok_longitude,
                                      tok_longitudeHemi.p[0]);
            nmea_reader_update_altitude(r, tok_altitude, tok_altitudeUnits);
            r->sv_status.num_used_svs = str2int(tok_usedInfix.p, tok_usedInfix.end);
        }

    } else if ( !memcmp(tok.p, "GLL", 3) ) {
		D("GLL parser IN");
        Token  tok_fixstaus      = nmea_tokenizer_get(tzer,6);
        if (tok_fixstaus.p[0] == 'A') 
        {
            Token  tok_latitude      = nmea_tokenizer_get(tzer,1);
            Token  tok_latitudeHemi  = nmea_tokenizer_get(tzer,2);
            Token  tok_longitude     = nmea_tokenizer_get(tzer,3);
			Token  tok_longitudeHemi = nmea_tokenizer_get(tzer,4);
			Token  tok_time          = nmea_tokenizer_get(tzer,5);
			nmea_reader_update_time(r, tok_time);
			nmea_reader_update_latlong(r, tok_latitude,
                                              tok_latitudeHemi.p[0],
                                              tok_longitude,
                                              tok_longitudeHemi.p[0]);
		}
 
    } else if ( !memcmp(tok.p, "GSA", 3) ){
	    D("GSA parser IN");
		Token  tok_fixStatus   = nmea_tokenizer_get(tzer, 2);
		int i;
		if (tok_fixStatus.p[0] != '\0' && tok_fixStatus.p[0] != '1') 
		{
		    Token  tok_accuracy      = nmea_tokenizer_get(tzer, 15);
			nmea_reader_update_accuracy(r, tok_accuracy);
			r->sv_status.used_in_fix_mask = 0ul;
			for (i = 3; i <= 14; ++i)
			{
                Token  tok_prn  = nmea_tokenizer_get(tzer, i);
                int prn = str2int(tok_prn.p, tok_prn.end);
                if (prn > 0 && prn < 32)
                {
                    r->sv_status.used_in_fix_mask |= (1ul << (prn-1));
                    r->sv_status_changed = 1;
                    D("%s: fix mask is %s, %d", __FUNCTION__, r->sv_status.used_in_fix_mask);
                }
            }

        }

    } else if ( !memcmp(tok.p, "GSV", 3) ) {
		D("GSV parser IN");
		Token  tok_noSatellites  = nmea_tokenizer_get(tzer, 3);
		int    noSatellites = str2int(tok_noSatellites.p, tok_noSatellites.end);
		if (noSatellites > 0) 
        {
            Token  tok_noSentences   = nmea_tokenizer_get(tzer, 1);
			Token  tok_sentence      = nmea_tokenizer_get(tzer, 2);
			int sentence = str2int(tok_sentence.p, tok_sentence.end);
			int totalSentences = str2int(tok_noSentences.p, tok_noSentences.end);
			int curr;
			int i;
			if (sentence == 1) 
			{
                r->sv_status_changed = 0;
                r->sv_status.num_svs = 0;
            }
			curr = r->sv_status.num_svs;
			i = 0;
			while (i < 4 && r->sv_status.num_svs < noSatellites)
			{
                Token  tok_prn = nmea_tokenizer_get(tzer, i * 4 + 4);
                Token  tok_elevation = nmea_tokenizer_get(tzer, i * 4 + 5);
                Token  tok_azimuth = nmea_tokenizer_get(tzer, i * 4 + 6);
                Token  tok_snr = nmea_tokenizer_get(tzer, i * 4 + 7);
				r->sv_status.sv_list[curr].prn = str2int(tok_prn.p, tok_prn.end);
				r->sv_status.sv_list[curr].elevation = str2float(tok_elevation.p, tok_elevation.end);
				r->sv_status.sv_list[curr].azimuth = str2float(tok_azimuth.p, tok_azimuth.end);
				r->sv_status.sv_list[curr].snr = str2float(tok_snr.p, tok_snr.end);
				r->sv_status.num_svs += 1;
				curr += 1;
				i += 1;
			}

			if (sentence == totalSentences)
			{
                r->sv_status_changed = 1;
            }

            D("%s: GSV message with total satellites %d", __FUNCTION__, noSatellites);   

        }

    } else if ( !memcmp(tok.p, "RMC", 3) ) {
        D("RMC parser IN");
        Token  tok_fixStatus     = nmea_tokenizer_get(tzer,2);
		if (tok_fixStatus.p[0] == 'A')
		{
        
			Token  tok_time          = nmea_tokenizer_get(tzer,1);
            Token  tok_latitude      = nmea_tokenizer_get(tzer,3);
            Token  tok_latitudeHemi  = nmea_tokenizer_get(tzer,4);
            Token  tok_longitude     = nmea_tokenizer_get(tzer,5);
            Token  tok_longitudeHemi = nmea_tokenizer_get(tzer,6);
            Token  tok_speed         = nmea_tokenizer_get(tzer,7);
            Token  tok_bearing       = nmea_tokenizer_get(tzer,8);
            Token  tok_date          = nmea_tokenizer_get(tzer,9);
            nmea_reader_update_date( r, tok_date, tok_time );
            nmea_reader_update_latlong( r, tok_latitude,
                                              tok_latitudeHemi.p[0],
                                              tok_longitude,
                                              tok_longitudeHemi.p[0] );
  
            nmea_reader_update_bearing( r, tok_bearing );
            nmea_reader_update_speed  ( r, tok_speed );
        }

    } else if ( !memcmp(tok.p, "VTG", 3) ) {
        D("VTG parser IN");
		Token  tok_fixStatus     = nmea_tokenizer_get(tzer,9);
		if (tok_fixStatus.p[0] != '\0' && tok_fixStatus.p[0] != 'N')
		{
            Token  tok_bearing       = nmea_tokenizer_get(tzer,1);
			Token  tok_speed         = nmea_tokenizer_get(tzer,5);
			nmea_reader_update_bearing( r, tok_bearing );
			nmea_reader_update_speed  ( r, tok_speed );
		}

    } else if ( !memcmp(tok.p, "ZDA", 3) ) {
        D("ZDA parser IN");
        Token  tok_time;
		Token  tok_year  = nmea_tokenizer_get(tzer,4);
		if (tok_year.p[0] != '\0') 
		{
		    Token  tok_day   = nmea_tokenizer_get(tzer,2);
			Token  tok_mon   = nmea_tokenizer_get(tzer,3);
			nmea_reader_update_cdate( r, tok_day, tok_mon, tok_year );
		}
        tok_time  = nmea_tokenizer_get(tzer,1);
		if (tok_time.p[0] != '\0') 
		{
	        nmea_reader_update_time(r, tok_time);
		}
    } else {
        tok.p -= 2;
        D("unknown sentence '%.*s", tok.end-tok.p, tok.p);
    }

    if (!gps_state->first_fix &&
        gps_state->init == STATE_INIT &&
        r->fix.flags & GPS_LOCATION_HAS_LAT_LONG) 
    {

	    if (gps_state->callbacks.location_cb) 
        {
            gps_state->callbacks.location_cb( &r->fix );
            r->fix.flags = 0;
        }
        gps_state->first_fix = 1;
    }

#if 0
    if (r->fix.flags != 0) {
#if GPS_DEBUG
        char   temp[256];
        char*  p   = temp;
        char*  end = p + sizeof(temp);
        struct tm   utc;

        p += snprintf( p, end-p, "sending fix" );
        if (r->fix.flags & GPS_LOCATION_HAS_LAT_LONG) {
            p += snprintf(p, end-p, " lat=%g lon=%g", r->fix.latitude, r->fix.longitude);
        }
        if (r->fix.flags & GPS_LOCATION_HAS_ALTITUDE) {
            p += snprintf(p, end-p, " altitude=%g", r->fix.altitude);
        }
        if (r->fix.flags & GPS_LOCATION_HAS_SPEED) {
            p += snprintf(p, end-p, " speed=%g", r->fix.speed);
        }
        if (r->fix.flags & GPS_LOCATION_HAS_BEARING) {
            p += snprintf(p, end-p, " bearing=%g", r->fix.bearing);
        }
        if (r->fix.flags & GPS_LOCATION_HAS_ACCURACY) {
            p += snprintf(p,end-p, " accuracy=%g", r->fix.accuracy);
        }
        gmtime_r( (time_t*) &r->fix.timestamp, &utc );
        p += snprintf(p, end-p, " time=%s", asctime( &utc ) );
        D(temp);
#endif
        if (r->callback) {
            r->callback( &r->fix );
            r->fix.flags = 0;
        }
        else {
            D("no callback, keeping data until needed !");
        }
    }
#endif
}

static int fd_gpslog = -1;
static int fd_flag;
static char charFormart[30];

static void nmea_reader_addc( NmeaReader*  r, int  c )
{
    char p[30];
    char tmp[16];
    if (r->overflow) 
    {
        r->overflow = (c != '\n');
        return;
    }

    if (r->pos >= (int) sizeof(r->in)-1 ) 
    {
        r->overflow = 1;
        r->pos      = 0;
        return;
    }
    r->in[r->pos] = (char)c;
    r->pos       += 1;
    if (c == '\n') 
    {
        GPS_STATE_LOCK_FIX(gps_state);
        nmea_reader_parse( r );
        GPS_STATE_UNLOCK_FIX(gps_state);
        if(property_get("sys.gps.log", tmp, NULL) && strncmp(tmp,"on",2) == 0)
//    	  if(1)
	    {
		//	if ((fd_gpslog == 0 && !fd_flag) || fd_gpslog == -1)
            if (fd_gpslog == -1)
            {
	            time_t now;
	            struct tm *timenow;
                time(&now);
	            timenow = (struct tm*)localtime(&now);
	            sprintf(charFormart,"\/sdcard\/%4.4d-%2.2d-%2.2d-%2.2d%2.2d.log",
	                timenow->tm_year + 1900,
		            timenow->tm_mon + 1,
		            timenow->tm_mday,
		            timenow->tm_hour,
		            timenow->tm_min
		            );
	            fd_gpslog = open(charFormart,O_WRONLY|O_CREAT|O_APPEND);
	            if (fd_gpslog == -1) DFR("open log file failed\n");
//				if (fd_gpslog == -1) DFR(tmp);
	       }
	       DFR("fd_gpslog",fd_gpslog);
           if (fd_gpslog == -1) goto xxx;
//			sprintf(p, "time = %llu\n", r->fix.timestamp);
//			write(fd_gpslog, p, strlen(p) + 1); 
		   write(fd_gpslog, r->in, r->pos); 
        } else {
            if(fd_gpslog != -1)  
            {
                close(fd_gpslog);fd_gpslog = -1;
            }
        }
xxx:
        r->pos = 0;
//		DFR(tmp);
    }
}

/*****************************************************************/
/*****************************************************************/
/*****                                                       *****/
/*****       C O N N E C T I O N   S T A T E *****/
/*****                                                       *****/
/*****************************************************************/
/*****************************************************************/

/* commands sent to the gps thread */
enum {
    CMD_QUIT  = 0,
    CMD_START = 1,
    CMD_STOP  = 2
};

static void gps_state_update_fix_freq(GpsState *s, int fix_freq)
{
    D("gps_state_update_fix_freq In");
    s->fix_freq = fix_freq;
    D("gps_state_update_fix_freq out");
    return;

}


static void gps_state_done( GpsState*  s )
{
    D("gps_state_done In");
// tell the thread to quit, and wait for it
    char   cmd = CMD_QUIT;
    void*  dummy;
    int ret;
    DFR("gps send quit command");
    do { 
        ret=write( s->control[0], &cmd, 1 ); 
    } while (ret < 0 && errno == EINTR);

    DFR("gps waiting for command thread to stop");
    pthread_join(s->thread, &dummy);
/* Timer thread depends on this state check */
    s->init = STATE_QUIT;
    s->fix_freq = -1;
// close the control socket pair
    close( s->control[0] ); s->control[0] = -1;
    close( s->control[1] ); s->control[1] = -1;
// close connection to the QEMU GPS daemon
    close( s->fd ); s->fd = -1;
    sem_destroy(&s->fix_sem);
    memset(s, 0, sizeof(*s));
    DFR("gps deinit complete");
    D("gps_state_done out");
}


static void gps_state_start( GpsState*  s )
{
    D("gps_state_start In");
    char  cmd = CMD_START;
    int   ret;
    do { 
	    ret=write( s->control[0], &cmd, 1 ); 
    } while (ret < 0 && errno == EINTR);
    if (ret != 1)
    {
        D("%s: could not send CMD_START command: ret=%d: %s",
           __FUNCTION__, ret, strerror(errno));
    }
    D("gps_state_start out");
}


static void gps_state_stop( GpsState*  s )
{
    D("gps_state_stop In");
    char  cmd = CMD_STOP;
    int   ret;
    do { 
        ret=write( s->control[0], &cmd, 1 ); 
    } while (ret < 0 && errno == EINTR);

    if (ret != 1)
     {
         D("%s: could not send CMD_STOP command: ret=%d: %s",
         __FUNCTION__, ret, strerror(errno));
     } 
    D("gps_state_stop out");
}


static int epoll_register( int  epoll_fd, int  fd )
{
    struct epoll_event  ev;
    int                 ret, flags;
/* important: make the fd non-blocking */
    flags = fcntl(fd, F_GETFL);
    fcntl(fd, F_SETFL, flags | O_NONBLOCK);
    ev.events  = EPOLLIN;
    ev.data.fd = fd;
    do {
           ret = epoll_ctl( epoll_fd, EPOLL_CTL_ADD, fd, &ev );
       } while (ret < 0 && errno == EINTR);
    return ret;
}

static int epoll_deregister( int  epoll_fd, int  fd )
{
    D("epoll_deregister In");
    int  ret;
    do {
           ret = epoll_ctl( epoll_fd, EPOLL_CTL_DEL, fd, NULL );
       } while (ret < 0 && errno == EINTR);
    D("epoll_deregister out");
    return ret;
}

/* this is the main thread, it waits for commands from gps_state_start/stop and,
 * when started, messages from the QEMU GPS daemon. these are simple NMEA sentences
 * that must be parsed to be converted into GPS fixes sent to the framework
 */
static int gps_power_on(void);
static int gps_power_off(void);

static struct timeval get_time_now(void);
static struct timeval t0;
static struct timeval t1;
static unsigned long time_div;
static unsigned int first_fix_location;

static void* gps_state_thread( void*  arg )
{
    D("gps_state_thread IN");
    GpsState*   state = (GpsState*) arg;
    NmeaReader  *reader;
    int         epoll_fd   = epoll_create(2);
    int         started    = 0;
    int         gps_fd     = state->fd;
    int         control_fd = state->control[1];
    reader = &state->reader;
    nmea_reader_init( reader );
// register control file descriptors for polling
    epoll_register( epoll_fd, control_fd );
    epoll_register( epoll_fd, gps_fd );
    D("gps thread running");
    gps_power_on();
    t0 = get_time_now();
    // now loop
    for (;;) 
    {
        struct epoll_event   events[2];
        int                  ne, nevents;
        nevents = epoll_wait( epoll_fd, events, 2, -1 );
        if (nevents < 0) 
        {
            if (errno != EINTR)
            {
                LOGE("epoll_wait() unexpected error: %s", strerror(errno));
            }
           
            continue;
        }
// D("gps thread received %d events", nevents);
        for (ne = 0; ne < nevents; ne++) 
        {
            if ((events[ne].events & (EPOLLERR|EPOLLHUP)) != 0) 
            {
                LOGE("EPOLLERR or EPOLLHUP after epoll_wait() !?");
                goto Exit;
            }
            if ((events[ne].events & EPOLLIN) != 0) 
            {
                int  fd = events[ne].data.fd;
                if (fd == control_fd)
                {
                    char  cmd = 255;
                    int   ret;
                    D("gps control fd event");
                    do {
                           ret = read( fd, &cmd, 1 );
                    } while (ret < 0 && errno == EINTR);

                    
                    if (cmd == CMD_QUIT) 
                          {
                        D("gps thread quitting on demand");
                        goto Exit;
                    } else if (cmd == CMD_START)
                    {
                        if (!started) 
                        {
                            D("gps thread starting  location_cb=%p", state->callbacks.location_cb);
                            started = 1;
//  gps_dev_start(gps_fd);
                            GPS_STATUS_CB(state->callbacks, GPS_STATUS_SESSION_BEGIN);
                            state->init = STATE_START;
                            if ( pthread_create( &state->tmr_thread, NULL, gps_timer_thread, state ) != 0 ) 
                            {
                                LOGE("could not create gps timer thread: %s", strerror(errno));
                                started = 0;
                                state->init = STATE_INIT;
                                goto Exit;
                            }

                         }
                    } else if (cmd == CMD_STOP) 
                    {
                        if (started) 
                        {
                            void *dummy;
                            D("gps thread stopping");
                            started = 0;
// gps_dev_stop(gps_fd);
                            state->init = STATE_INIT;
                            pthread_join(state->tmr_thread, &dummy);
                            GPS_STATUS_CB(state->callbacks, GPS_STATUS_SESSION_END);
                        }
                    }
                } else if (fd == gps_fd)
                {
                    char buf[512];
                    int  nn, ret;
                    do {
                        ret = read( fd, buf, sizeof(buf) );
                    } while (ret < 0 && errno == EINTR);

			        if (ret < 0) 
                    {
			            if (errno == EINTR)
                        {
   				            continue;
                                        }
			            } 
//D("received %d bytes: %s", ret, buf);
                        if (ret > 0)
			            {
                            for (nn = 0; nn < ret; nn++)
                            {
                                nmea_reader_addc( reader, buf[nn] );
							}
                            
						}
                       
////////////////////////
#if 0
	               if (state->reader.fix.flags !=0 && !first_fix_location)
					{
						char *p;
						int fd;
						DFR("first fixing =============================================================\n");
						first_fix_location = 1;
						t1 = get_time_now();
						time_div = t1.tv_sec * 1000 + t1.tv_usec / 1000 -(t0.tv_sec * 1000 + t0.tv_usec / 1000 );
						fd = open("/data/starting_time",O_WRONLY|O_CREAT|O_APPEND);
					//	if (fd == -1) goto xxx;
						DFR("first fixing div_time is %f ms\n", time_div);
						DFR("first fixing div_time is %f ms\n", time_div);
						DFR("first fixing div_time is %f ms\n", time_div);
						DFR("first fixing div_time is %d ms\n", time_div);
						asprintf(&p, "time = %d\n", time_div);
						write(fd, p, strlen(p)+1); 
						xxx:
						close(fd);
					}
					////////////////////////
#endif


                     // D("gps fd event end");
			   //D("gps_state_thread out");

                  } else
                  {
                      LOGE("epoll_wait() returned unkown fd %d ?", fd);
                  }
             }
        }
    }
Exit:
	gps_power_off();
	if(fd_gpslog != -1)
	close(fd_gpslog);
	fd_gpslog = -1;
      return NULL;
}

static void* gps_timer_thread( void*  arg )
{
    D("gps_time_thread IN");
    GpsState *state = (GpsState *)arg;
    DFR("gps entered timer thread");
    do {
        D ("gps timer exp");
        GPS_STATE_LOCK_FIX(state);
		if (state->reader.fix.flags != 0)
       	{
            D("gps fix cb: 0x%x", state->reader.fix.flags);
          	if (state->callbacks.location_cb) 
           	{
	     	    state->callbacks.location_cb( &state->reader.fix );
	            state->reader.fix.flags = 0;
	            state->first_fix = 1;
	        }
	        if (state->fix_freq == 0) 
            {
	            state->fix_freq = -1;
	        }
         }

    	if (state->reader.sv_status_changed != 0) 
        {
            D("gps sv status callback");
            if (state->callbacks.sv_status_cb) 
            {
	            state->callbacks.sv_status_cb( &state->reader.sv_status );
	            state->reader.sv_status_changed = 0;
	        }
        }
        GPS_STATE_UNLOCK_FIX(state);
	    sleep(state->fix_freq);
    } while(state->init == STATE_START);
	
	DFR("gps timer thread destroyed");
    D("gps_time_thread out");
    return NULL;
}

int gps_open(void)
{
    D("gps_open IN");
    struct termios tio;
    int tty_fd;
    speed_t speed = B4800;
    if((tty_fd = open("/dev/s3c2410_serial2", O_RDWR)) < 0)
    {
        D("gps semaphore initialization failed! errno ");
        return -1;
    }
    tio.c_iflag = IGNBRK | IGNPAR;
    tio.c_cflag = CLOCAL | CREAD | CS8 | HUPCL | CRTSCTS;
    tio.c_oflag = 0;
    tio.c_lflag = 0;
    tio.c_cc[VMIN] = 1;
    tio.c_cc[VTIME] = 10;
    cfsetispeed(&tio, speed);
    cfsetospeed(&tio, speed);
    tcsetattr(tty_fd, TCSANOW, &tio);
    tcsetattr(tty_fd, TCSANOW, &tio);
    D("gps_open out");
    return tty_fd;
}
static void gps_state_init( GpsState*  state )
{
    D("gps_state_init In");
//char prop[PROPERTY_VALUE_MAX];
//char device[256];
    int  ret;
    int  done = 0;
    struct sigevent tmr_event;
    state->init       = STATE_INIT;
    state->control[0] = -1;
    state->control[1] = -1;
    state->fd         = -1;
    state->fix_freq   = -1;
    state->first_fix  = 0;
    if (sem_init(&state->fix_sem, 0, 1) != 0) 
    {
        D("gps semaphore initialization failed! errno ");
        D("gps_state_init out");
        return;
    }
    state->fd = gps_open();
  //look for a kernel-provided device name
  // if (property_get("ro.kernel.android.gps",prop,"") == 0) {
  //    D("no kernel-provided gps device name");
  //   return;
   // }
  //if ( snprintf(device, sizeof(device), "/dev/%s", prop) >= (int)sizeof(device) ) {
  //   LOGE("gps serial device name too long: '%s'", prop);
  //  return;
  //    }
    if (state->fd < 0) 
    {
      LOGE("could not open gps serial device ");
      return;
    }
 // disable echo on serial lines
 //   if ( isatty( state->fd ) ) {
  //      struct termios  ios;
   //     tcgetattr( state->fd, &ios );
    //    ios.c_lflag = 0;  /* disable ECHO, ICANON, etc... */
     //   ios.c_oflag &= (~ONLCR); /* Stop \n -> \r\n translation on output */
      //  ios.c_iflag &= (~(ICRNL | INLCR)); /* Stop \r -> \n & \n -> \r translation on input */
       // ios.c_iflag |= (IGNCR | IXOFF);  /* Ignore \r & XON/XOFF on input */
      //  tcsetattr( state->fd, TCSANOW, &ios );
   // }

    if ( socketpair( AF_LOCAL, SOCK_STREAM, 0, state->control ) < 0 ) 
    {
        LOGE("could not create thread control socket pair: %s", strerror(errno));
        goto Fail;
    }
    if ( pthread_create( &state->thread, NULL, gps_state_thread, state ) != 0 ) 
    {
        LOGE("could not create gps thread: %s", strerror(errno));
        goto Fail;
    }
    D("gps_state_init out");
    return;
    Fail:
        gps_state_done( state );
}

/*****************************************************************/
/*****************************************************************/
/*****                                                       *****/
/***         I N T E R F A C E                ***/
/*****                                                       *****/
/*****************************************************************/
/*****************************************************************/

static int vimm_gps_init(GpsCallbacks* callbacks)
{
    D("vimm_gps_init IN");
    GpsState*  s = _gps_state;
    if (!s->init)
    {
        gps_state_init(s);
    }
    if (s->fd < 0)
    {
        return -1;
    }
    s->callbacks = *callbacks;
    D("vimm_gps_init out");
    return 0;
}

static void vimm_gps_cleanup(void)
{
    D("vimm_gps_cleanup IN");
    GpsState*  s = _gps_state;
    if (s->init)
    {
        gps_state_done(s);
    }
    D("vimm_gps_cleanup out");
}

static int vimm_gps_start()
{
 	D("vimm_gps_start IN");
 	GpsState*  s = _gps_state;
 	if (!s->init) 
 	{
        DFR("%s: called with uninitialized state !!", __FUNCTION__);
        return -1;
    }
    D("%s: called", __FUNCTION__);
    gps_state_start(s);
    D("vimm_gps_start out");
    return 0;
}


static int vimm_gps_stop()
{
    D("vimm_gps_stop IN");
	GpsState*  s = _gps_state;
	if (!s->init) 
	{
	    DFR("%s: called with uninitialized state !!", __FUNCTION__);
    	return -1;
  	}
  	D("%s: called", __FUNCTION__);
  	gps_state_stop(s);
  	D("vimm_gps_stop out");
  	return 0;
}

static void vimm_gps_set_fix_frequency(int freq)
{
  	D("vimm_gps_set_fix_frequency IN");
  	GpsState*  s = _gps_state;
  	if (!s->init) 
  	{
    	DFR("%s: called with uninitialized state !!", __FUNCTION__);
    	return;
  	}
  	D("%s: called", __FUNCTION__);
  	s->fix_freq = (freq <= 0) ? 1 : freq;
  	D("gps fix frquency set to %d secs", freq);
  	D("vimm_gps_set_fix_frequency out");
}

static int vimm_gps_inject_time(GpsUtcTime time, int64_t timeReference, int uncertainty)
{
    return 0;
}

static void vimm_gps_delete_aiding_data(GpsAidingData flags)
{

}

static int vimm_gps_set_position_mode(GpsPositionMode mode, int fix_frequency)
{
	D("vimm_gps_set_position_mode IN");
    GpsState*  s = _gps_state;
    /*only standalone supported for now.
    if (mode != GPS_POSITION_MODE_STANDALONE)
        return -1;*/
    if (!s->init || fix_frequency < 0) 
    {
        D("%s: called with uninitialized state !!", __FUNCTION__);
        return -1;
    }
    s->fix_freq = fix_frequency;
    D("gps fix frquency set to %d secs", fix_frequency);
    D("vimm_gps_set_position_mode out");
    return 0;
}


/* guanxiaowei 20100817 begin: add this function to inject location  */
static int vimm_gps_inject_location(double latitude, double longitude, float accuracy)
{
    return 0;
}
/* guanxiaowei 20100817 begin: add this function to inject location */

static const void*
vimm_gps_get_extension(const char* name)
{
    return NULL;
}

static const GpsInterface  hardwareGpsInterface = {
    vimm_gps_init,
    vimm_gps_start,
    vimm_gps_stop,
    vimm_gps_cleanup,
    vimm_gps_inject_time,
/* guanxiaowei 20100817 begin: add this function to fit GPSInterface */
    vimm_gps_inject_location,
/* guanxiaowei 20100817 begin: add this function to fit GPSInterface */
    vimm_gps_delete_aiding_data,
    vimm_gps_set_position_mode,
    vimm_gps_get_extension,
};

const GpsInterface* gps_get_hardware_interface()
{
    return &hardwareGpsInterface;
}

/***********************************************************************************/
/***********************************************************************************/
/********                                                               ************/
/*****        device_power                            *********/
/********                                                               ************/
/********                                                               ************/
/***********************************************************************************/
/***********************************************************************************/

static int gps_power_on(void)
{
    D("gps_power_on IN");
    int fd;
    if((fd = open("/proc/gps_proc", O_RDWR)) < 0)
    {
        DFR("open gps_proc failed\n");
        return -1;
    }
    write(fd, "on",3);
    close(fd);
    D("gps_power_on out");
    return 0;
}

static int gps_power_off(void)
{
    D("gps_power_off IN");
    int fd;
    if((fd = open("/proc/gps_proc", O_RDWR)) < 0)
    {
        DFR("open gps_proc failed\n");
        return -1;
    }
    write(fd, "off",4);
    close(fd);
    D("gps_power_off out");
    return 0;
}

static struct timeval get_time_now(void)
{
    struct timeval t;
    gettimeofday(&t,0);
    return t;
}

