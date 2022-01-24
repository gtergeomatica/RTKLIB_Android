#include "../rtklib.h"
#include<stdbool.h>

#define AGSYNC1    78        /* gter message sync code 1 "R"  N 78*/
#define AGSYNC2    66        /* gter message sync code 2 "a"  B 66*/

#define SPEED_OF_LIGHT 299792458.0  // [m/s]
#define GPS_WEEKSEC 604800  // Number of seconds in a week
#define NS_TO_S 1e-9 
#define BDST_TO_GPST 14 //Leap seconds difference between BDST and GPST
#define DAYSEC 86400
#define CURRENT_GPS_LEAP_SECOND 18
#define GLOT_TO_UTC 10800  // Time difference between GLOT and UTC in seconds
#define ADR_STATE_VALID 0x00000001
#define STATE_GAL_E1C_2ND_CODE_LOCK 0x00000800
#define STATE_GAL_E1B_PAGE_SYNC 0x00001000
#define STATE_CODE_LOCK 0x00000001
#define STATE_TOW_DECODED 0x00000008
#define STATE_GLO_TOD_DECODED 0x00000080
#define STATE_MSEC_AMBIGUOUS 0x00000010
#define STATE_GAL_E1BC_CODE_LOCK 0x00000400
#define STATE_GAL_E1C_2ND_CODE_LOCK 0x00000800


#define U1(p) (*((uint8_t *)(p)))
#define I1(p) (*((int8_t  *)(p)))
static uint16_t U2(uint8_t* p) { uint16_t u; memcpy(&u, p, 2); return u; }
static uint32_t U4(uint8_t* p) { uint32_t u; memcpy(&u, p, 4); return u; }

typedef struct {

    char typemeas[3];
    long long int utcTimeMillis;
    long long int TimeNanos;
    signed long long int FullBiasNanos;
    double BiasNanos;
    int TimeOffsetNanos;
    int State;
    int Svid;
    int ConstellationType;
    long long int ReceivedSvTimeNanos;
    double AccumulatedDeltaRange;
    double Cn0;
    double CarrierFrequencyHz;
    int ADRState;
    double PseudorangeRateMetersPerSecond;

}androidgnssmeas;

static double asc2dbl(int digits, uint8_t* input)
{
    double retVal = 0.0;
    for (int i = 0; i < digits; i++)
    {
        double tmp = (input[i] - 48);
        for (int j = 1; j < digits - i; j++)
            tmp *= 10.0;

        retVal += tmp;
    }
    return retVal;
}

static long asc2long(int digits, uint8_t* input)
{
    long retVal = 0;
    for (int i = 0; i < digits; i++)
    {
        long tmp = (input[i] - 48);
        for (int j = 1; j < digits - i; j++)
            tmp *= 10;

        retVal += tmp;
    }
    return retVal;
}

static double R8u(uint8_t* p)
{
    double retVal = 0;
    for (int i = 0; i < 8; i++)
        retVal += (p[i] * pow(16, i * 2));

    return retVal;
}

static double R8s(uint8_t* p)
{
    double fact = 1.0;
    double delta = 0.0;
    if ((p[7] & 128) == 128)
    {
        for (int i = 0; i < 8; i++)
            p[i] = 255 - p[i];

        fact = -1.0;
        delta = 1.0;
    }

    double retVal = 0;
    for (int i = 0; i < 8; i++)
        retVal += (p[i] * pow(16, i * 2));

    return (retVal + delta) * fact;
}

static int sync_gterAndroid(uint8_t* buff, uint8_t data)
{
    // code here to sync stream with start of next message
    buff[0] = buff[1]; buff[1] = data;
    return buff[0] == AGSYNC1 && buff[1] == AGSYNC2;


}

//LB: series of function needed to compute pseudorange and carrier-phase observation

time_t newDateTime(const int year, const int month, const int date, const int hrs, const int min, const int sec) {
	time_t time;
	struct tm tmStruct = { 0 };
	tmStruct.tm_mday = date;
	tmStruct.tm_mon = month - 1;
	tmStruct.tm_year = year - 1900;
	tmStruct.tm_hour = hrs;
	tmStruct.tm_min = min;
	tmStruct.tm_sec = sec;
	time = mktime(&tmStruct);
	if (time == -1) {
		printf("Data non supportata.");
		exit(1);
	}
	return time;
}

char* formatDateTime(const time_t mytime) {
	/*Used to printf date object*/
	char* dateTimeStr = malloc(20 * sizeof(char));
	struct tm date = *localtime(&mytime);
	sprintf(dateTimeStr, "%.2d/%.2d/%4d %.2d:%.2d:%.2d", date.tm_mday, date.tm_mon + 1, date.tm_year + 1900, date.tm_hour, date.tm_min, date.tm_sec);
	return dateTimeStr;
}

double check_week_crossover(double tRxSeconds, double tTxSeconds)
{
	/*
	Checks time propagation time for week crossover
		: param tRxSeconds : received time in seconds of week
		: param tTxSeconds : transmitted time in seconds of week
		: return : corrected propagation time
	*/
	int del_sec = 0;
	double tau = 0, rho_sec = 0;
	tau = tRxSeconds - tTxSeconds;
	if (tau > GPS_WEEKSEC / 2)
	{
		del_sec = round(tau / GPS_WEEKSEC) * GPS_WEEKSEC;
		rho_sec = tau - del_sec;
		if (rho_sec > 10) {
			tau = 0.0;
		}
		else {
			tau = rho_sec;
		}
	}
	return tau;
}

double glot_to_gpst(time_t gpst_current_epoch, double tod_seconds)
{
	/*
	 Converts GLOT to GPST
	:param gpst_current_epoch: Current epoch of the measurement in GPST
	:param tod_seconds: Time of days as number of seconds
	:return: Time of week in seconds
	*/

	// Get the GLONASS epoch given the current GPS time
	struct tm tmStruct, tmStructtod;
	double tod_sec_frac, tod_sec, tow_sec;
	time_t glo_epoch, glo_td, glo_tod;
	int day_of_week_sec;

	tod_sec_frac = modf(tod_seconds, &tod_sec);

	tmStruct = *localtime(&gpst_current_epoch);
	tmStruct.tm_hour += 3;
	tmStruct.tm_sec -= CURRENT_GPS_LEAP_SECOND;
	glo_epoch = mktime(&tmStruct);

	tmStruct = *localtime(&glo_epoch);
	tmStruct.tm_hour = 0;
	tmStruct.tm_min = 0;
	tmStruct.tm_sec = 0;
	tmStruct.tm_sec += tod_sec;
	glo_tod = mktime(&tmStruct); //LB: maybe this passage doent have sense

	//The day of week in seconds needs to reflect the time passed before the current day starts
	day_of_week_sec = tmStruct.tm_wday * DAYSEC;

	tow_sec = day_of_week_sec + tod_seconds - GLOT_TO_UTC + CURRENT_GPS_LEAP_SECOND;

	return tow_sec;

}

const char* get_constellation(androidgnssmeas gnssdata) {
	char constellation[10] = "A";

	if (gnssdata.ConstellationType == 1) {
		strcpy(constellation, "G");
		return constellation;
	}
	else if (gnssdata.ConstellationType == 2) {
		strcpy(constellation, "S");
		return constellation;
	}
	else if (gnssdata.ConstellationType == 3) {
		strcpy(constellation, "R");
		return constellation;
	}
	else if (gnssdata.ConstellationType == 4) {
		strcpy(constellation, "J");
		return constellation;
	}
	else if (gnssdata.ConstellationType == 5) {
		strcpy(constellation, "C");
		return constellation;
	}
	else if (gnssdata.ConstellationType == 6) {
		strcpy(constellation, "E");
		return constellation;
	}
	else if (gnssdata.ConstellationType == 7) {
		strcpy(constellation, "I");
		return constellation;
	}

}

const char* getSatID(androidgnssmeas gnssdata) {

	char satID[100] = "A";

	char number[100] = { 0 };
	_itoa(gnssdata.Svid, number, 10);

	strcpy(satID, get_constellation(gnssdata));

	strcat(satID, number);
	return satID;
}

int get_rnx_band_from_freq(double frequency)
{
	double ifreq = frequency / (10.23E6);
	int iifreq = ifreq;
	if (ifreq >= 154) {
		return 1;
	}
	else if (iifreq == 115) {
		return 5;
	}
	else if (iifreq == 153) {
		return 2;
	}
	else {
		printf("Invalid frequency detected\n");
		return -1;
	}

}

const char* get_rnx_attr(int band, char constellation, int state)
{
	char attr[10] = "1C";

	//Make distinction between GAL E1Cand E1B code
	if (band == 1 && constellation == 'E') {
		if ((state & STATE_GAL_E1C_2ND_CODE_LOCK == 0) && (state & STATE_GAL_E1B_PAGE_SYNC != 0))
		{
			strcpy(attr, "1B");
		}
	}
	else if (band == 5) {
		strcpy(attr, "5Q");
	}
	else if (band == 2 && constellation == 'C') {
		strcpy(attr, "2I");
	}
	return attr;
}

const char* get_obs_code(androidgnssmeas gnssdata)
{
	int band, freq;
	char constellation[10] = "A";
	char attr[10] = "A";
	char obscode[10] = "C";
	strcpy(constellation, get_constellation(gnssdata));
	freq = gnssdata.CarrierFrequencyHz;
	band = get_rnx_band_from_freq(freq);
	strcpy(attr, get_rnx_attr(band, constellation, gnssdata.State));

	strcat(obscode, attr);

	return obscode;
}

double get_frequency(androidgnssmeas gnssdata)
{
	double freq;
	if (gnssdata.CarrierFrequencyHz == 0) //LB: check how to parse null values: not sure this case will ever happen
	{
		freq = 154 * 10.24E6;
	}
	else {
		freq = gnssdata.CarrierFrequencyHz;
	}
	return freq;
}

void check_trck_state(androidgnssmeas gnssdata)
{
	double freq = get_frequency(gnssdata);
	int freq_band = get_rnx_band_from_freq(freq);

	if (gnssdata.ConstellationType == 1 || gnssdata.ConstellationType == 2 || gnssdata.ConstellationType == 4 || gnssdata.ConstellationType == 5)
	{
		if ((gnssdata.State & STATE_CODE_LOCK) == 0)
			printf("State %i, hase STATE CODE LOCK not valid\n", gnssdata.State);
		else if ((gnssdata.State & STATE_TOW_DECODED) == 0)
			printf("State %i, has  STATE TOW DECODED not valid\n", gnssdata.State);
		else if ((gnssdata.State & STATE_MSEC_AMBIGUOUS) != 0)
			printf("State %i, has  STATE_MSEC_AMBIGUOUS not valid\n", gnssdata.State);
	}
	else if (gnssdata.ConstellationType == 3)
	{
		if ((gnssdata.State & STATE_CODE_LOCK) == 0)
			printf("State %i, has STATE CODE LOCK not valid\n", gnssdata.State);
		else if ((gnssdata.State & STATE_GLO_TOD_DECODED) == 0)
			printf("State %i, has STATE_GLO_TOD_DECODED not valid\n", gnssdata.State);
		else if ((gnssdata.State & STATE_MSEC_AMBIGUOUS) != 0)
			printf("State %i, has  STATE_MSEC_AMBIGUOUS not valid\n", gnssdata.State);
	}
	else if (gnssdata.ConstellationType == 6)
	{
		if (freq_band == 1)
		{
			if ((gnssdata.State & STATE_GAL_E1BC_CODE_LOCK) == 0)
				printf("State %i, has STATE GAL E1BC CODE LOCK not valid\n", gnssdata.State);
			else if ((gnssdata.State & STATE_GAL_E1C_2ND_CODE_LOCK) == 0) //State value indicates presence of E1B code
			{
				if ((gnssdata.State & STATE_TOW_DECODED) == 0)
					printf("State %i, has  STATE TOW DECODED not valid\n", gnssdata.State);
				else if ((gnssdata.State & STATE_MSEC_AMBIGUOUS) != 0)
					printf("State %i, has  STATE_MSEC_AMBIGUOUS not valid\n", gnssdata.State);
			}
			else //State value indicates presence of E1C code
			{
				if ((gnssdata.State & STATE_GAL_E1C_2ND_CODE_LOCK) == 0)
					printf("State %i, has STATE_GAL_E1C_2ND_CODE_LOCK not valid\n", gnssdata.State);
				else if ((gnssdata.State & STATE_MSEC_AMBIGUOUS) != 0)
					printf("State %i, has  STATE_MSEC_AMBIGUOUS not valid\n", gnssdata.State);
			}
		}
		else if (freq_band == 5)
		{
			if ((gnssdata.State & STATE_CODE_LOCK) == 0)
				printf("State %i, has STATE CODE LOCK not valid\n", gnssdata.State);
			else if ((gnssdata.State & STATE_GLO_TOD_DECODED) == 0)
				printf("State %i, has STATE_GLO_TOD_DECODED not valid\n", gnssdata.State);
			else if ((gnssdata.State & STATE_MSEC_AMBIGUOUS) != 0)
				printf("State %i, has  STATE_MSEC_AMBIGUOUS not valid\n", gnssdata.State);
		}

	}
	else
		printf("Constellation Type %i, is invalid or not implemented\n", gnssdata.ConstellationType);

}

double computePseudorange(androidgnssmeas gnssdata, float psdrgBias)
{
	int gpsweek = 0;
	double psrange, local_est_GPS_time = 0, gpssow = 0, T_Rx_seconds = 0, T_Tx_seconds = 0, tau = 0, Tod_secs = 0;
	struct tm tmStruct;
	time_t gpst_epoch, gpstime;
	gpstime = newDateTime(1980, 01, 06, 00, 00, 00);

	//compute Receiver time
	gpsweek = floor(-gnssdata.FullBiasNanos * NS_TO_S / GPS_WEEKSEC);
	local_est_GPS_time = gnssdata.TimeNanos - (gnssdata.FullBiasNanos + gnssdata.BiasNanos);

	gpssow = local_est_GPS_time * NS_TO_S - gpsweek * GPS_WEEKSEC;

	tmStruct = *localtime(&gpstime);
	tmStruct.tm_mday += gpsweek * 7;
	tmStruct.tm_sec += gpssow;
	gpst_epoch = mktime(&tmStruct); //LB: Nanoseconds not handled

	if (gpst_epoch == -1) {
		printf("Data non supportata.");
		exit(1);
	}

	T_Rx_seconds = gpssow - gnssdata.TimeOffsetNanos * NS_TO_S;

	//compute satellite emission time

	//check trck status
	//check_trck_state(gnssdata);

	//split cases depending on different constellations
	if (gnssdata.ConstellationType == 2)
	{
		printf("ERROR: Pseudorange computation not supported for SBAS\n");
		return -1;
	}
	else if (gnssdata.ConstellationType == 3)
	{
		//GLONASS
		Tod_secs = gnssdata.ReceivedSvTimeNanos * NS_TO_S;
		T_Tx_seconds = glot_to_gpst(gpst_epoch, Tod_secs);
	}
	else if (gnssdata.ConstellationType == 5)
	{
		//BEIDOU
		T_Tx_seconds = gnssdata.ReceivedSvTimeNanos * NS_TO_S + BDST_TO_GPST;

	}
	else if (gnssdata.ConstellationType == 1 || gnssdata.ConstellationType == 6)
	{
		//GPS and GALILEO
		T_Tx_seconds = gnssdata.ReceivedSvTimeNanos * NS_TO_S;
	}
	else
	{
		printf("Case not implemented\n");
		return -1;
	}

	tau = check_week_crossover(T_Rx_seconds, T_Tx_seconds);
	psrange = tau * SPEED_OF_LIGHT;
	return psrange;
}

double computeCarrierPhase(androidgnssmeas gnssdata) {
	double cphase, wavelength;

	wavelength = SPEED_OF_LIGHT / get_frequency(gnssdata);

	if ((gnssdata.ADRState & ADR_STATE_VALID) == 0) {
		printf("ADR STATE not Valid --> cphase = 0.0 \n");
		cphase = 0.0;
		return cphase;
	}

	cphase = gnssdata.AccumulatedDeltaRange / wavelength;

	return cphase;
}

double computeDoppler(androidgnssmeas gnssdata)
{
	double doppler, wavelength;
	wavelength = SPEED_OF_LIGHT / get_frequency(gnssdata);
	doppler = -gnssdata.PseudorangeRateMetersPerSecond / wavelength;
	return doppler;
}

char* mystrsep(char** stringp, const char* delim)
{
	char* start = *stringp;
	char* p;

	p = (start != NULL) ? strpbrk(start, delim) : NULL;

	if (p == NULL)
	{
		*stringp = NULL;
	}
	else
	{
		*p = '\0';
		*stringp = p + 1;
	}

	return start;
}



static int decode_gterAndroid(raw_t* raw)
{
    // code here to decode android message

    //parse
 //   int row_count_r = 0;
 //   int col_count_r = 0;
 //   char* col = strtok(raw->buff, ",");

	const char delimiters[] = ",";
	char* running;
	char* currstrval; //current field read in the str	
	int ncolmeas = 30; //number of fields for single measurement
	int nmeas;
	

	running = raw->buff;

	//read the number of Raw meas sent for this epoch
	for (int i = 0; i < 2; i++) {
		currstrval = mystrsep(&running, delimiters);
		if (i == 1)
			nmeas = atoi(currstrval);
	}

	//read values in every Raw meas 
	androidgnssmeas* andrawdata = malloc(sizeof(androidgnssmeas)*nmeas);
	for (int n = 0; n < nmeas; n++) {

		for (int j = 0; j < ncolmeas; j++) {
			currstrval = mystrsep(&running, delimiters);
			if (j == 0)
				strcpy(andrawdata[n].typemeas, currstrval);
			else if (j == 1)
				andrawdata[n].utcTimeMillis = atoll(currstrval);
			else if (j == 2)
				andrawdata[n].TimeNanos = atoll(currstrval);
			else if (j == 5)
				andrawdata[n].FullBiasNanos = atoll(currstrval);
			else if (j == 6)
				sscanf(currstrval, "%lf", &andrawdata[n].BiasNanos);
			else if (j == 11)
				andrawdata[n].Svid = atoi(currstrval);
			else if (j == 12)
				andrawdata[n].TimeOffsetNanos = atoi(currstrval);
			else if (j == 13)
				andrawdata[n].State = atoi(currstrval);
			else if (j == 14)
				andrawdata[n].ReceivedSvTimeNanos = atoll(currstrval);
			else if (j == 16)
				sscanf(currstrval, "%lf", &andrawdata[n].Cn0);
			else if (j == 17)
				sscanf(currstrval, "%lf", &andrawdata[n].PseudorangeRateMetersPerSecond);
			else if (j == 19)
				andrawdata[n].ADRState = atoi(currstrval);
			else if (j == 20)
				sscanf(currstrval, "%lf", &andrawdata[n].AccumulatedDeltaRange);
			else if (j == 22)
				sscanf(currstrval, "%lf", &andrawdata[n].CarrierFrequencyHz);
			else if (j == 28)
				andrawdata[n].ConstellationType = atoi(currstrval);
		}
	}



	char* sat;
	char* code;
	float psdrgBias = 0;
	double psdrange,cphase,doppler;

	//sat = getSatID(andrawdata);

	//code = get_obs_code(andrawdata);

	//psdrange = computePseudorange(andrawdata, psdrgBias);

	//cphase = computeCarrierPhase(andrawdata);

	//doppler = computeDoppler(andrawdata);

	//printf("GTER DEBUG GNSS ANDROID: %s, %s", sat, code);









}

extern int input_gterAndroid(raw_t* raw, uint8_t data)
{
    trace(5, "input_gterAndroid: data=%02x\n", data);

    /* synchronize frame */
    if (raw->nbyte == 0) {
        if (!sync_gterAndroid(raw->buff, data)) return 0;
        raw->nbyte = 2;
        return 0;
    }
    raw->buff[raw->nbyte++] = data;

    //TODO:check on possible corrupted message?
    //if (raw->nbyte == 4) {
    //    if ((raw->len = U2(&raw->buff[2])) > MAXRAWLEN) { // warning: modify this
    //        trace(2, "ay length error: len=%d\n", raw->len);
    //        raw->nbyte = 0;
    //        return -1;
    //    }
    //}
    //if (raw->nbyte < 4 || raw->nbyte < raw->len) return 0;
    //raw->nbyte = 0;

    if (raw->buff[raw->nbyte - 1] != '\n')
        return 0;
    else if (raw->nbyte == 570 & raw->buff[568] == 'b') {

        raw->nbyte = 0;
        return -1;
    }
    raw->nbyte = 0;
    /* decode ublox raw message */
    return decode_gterAndroid(raw);
}