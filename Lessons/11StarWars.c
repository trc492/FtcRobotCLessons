#define NUM_FREQ                22
#define NUM_NOTES               84
#define DURATION_SCALE          8

int g_FreqTable[NUM_FREQ] =
{
    0,          //00: Rest
    262,        //01: Do
    294,        //02: Re
    330,        //03: Me
    349,        //04: Fa
    392,        //05: So
    440,        //06: La
    494,        //07: Ti
    0,          //08: Rest
    0,          //09: Rest
    0,          //10: Rest
    523,        //11: Do
    587,        //12: Re
    659,        //13: Me
    698,        //14: Fa
    784,        //15: So
    880,        //16: La
    988,        //17: Ti
    0,          //18: Rest
    0,          //19: Rest
    0,          //20: Rest
    1047        //21: Do
};

int g_Notes[NUM_NOTES] =
{
    5, 5, 5,
    11, 15,
    14, 13, 12, 21, 15,
    14, 13, 12, 21, 15,
    14, 13, 14, 12, 5, 5,
    11, 15,
    14, 13, 12, 21, 15,
    14, 13, 12, 21, 15,
    14, 13, 14, 12, 5, 5,
    6, 6, 14, 13, 12, 11,
    11, 12, 13, 12, 7, 5, 5,
    6, 6, 14, 13, 12, 11,
    15, 12, 5, 5,
    6, 6, 14, 13, 12, 11,
    11, 12, 13, 12, 7, 15, 15,
    21, 932, 831, 15, 14, 622, 12, 11,
    15
};

int g_Durations[NUM_NOTES] =
{
    2, 2, 2,
    12, 12,
    2, 2, 2, 12, 6,
    2, 2, 2, 12, 6,
    2, 2, 2, 12, 3, 3,
    12, 12,
    2, 2, 2, 12, 6,
    2, 2, 2, 12, 6,
    2, 2, 2, 12, 3, 3,
    9, 3, 3, 3, 3, 3,
    2, 2, 2, 6, 6, 3, 3,
    9, 3, 3, 3, 3, 3,
    6, 12, 3, 3,
    9, 3, 3, 3, 3, 3,
    2, 2, 2, 6, 6, 3, 3,
    3, 3, 3, 3, 3, 3, 3, 3,
    18
};

task main()
{
    int freq;

    while (true)
    {
        for (int i = 0; i < NUM_NOTES; i++)
        {
            freq = (g_Notes[i] < NUM_FREQ)?
                    g_FreqTable[g_Notes[i]]:
                    g_Notes[i];
            nxtDisplayTextLine(2, "[%02d]%4d,%d",
                               i, freq, g_Durations[i]);
            PlayTone(freq, g_Durations[i]*DURATION_SCALE);
            while (bSoundActive)
            {
                EndTimeSlice();
            }
        }
    }
}
