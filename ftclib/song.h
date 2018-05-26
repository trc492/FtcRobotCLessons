#if 0
/// Copyright (c) Titan Robotics Club. All rights reserved.
///
/// <module name="song.h" />
///
/// <summary>
///     This module contains the library functions for the song object.
/// </summary>
///
/// <remarks>
///     Environment: RobotC for Lego Mindstorms NXT.
/// </remarks>
#endif

#ifndef _SONG_H
#define _SONG_H

#pragma systemFile

#ifdef MOD_ID
    #undef MOD_ID
#endif
#define MOD_ID                  MOD_SONG

//
// Constants.
//
#define DEF_BAR_DURATION        1920

#define SONGO_REPEAT            0x0001
#define SONGO_PAUSE             0x0002

#define SONGF_STARTED           0x0001
#define SONGF_PARSE_ERROR       0x0002

//
// Macros.
//

//
// Type definitions.
//
typedef struct
{
    char  **ppszSong;
    char  **ppszCurrSeg;
    char   *pszNextNote;
    int     flags;
    int     barDuration;
    int     options;
    SM     *sm;
    int     evtType;
} SONG;

/**
 *  This function parses the note frequency from the song segment string.
 *
 *  @param pszNote Points to the current note string.
 *  @param len Specifies the current note string length.
 */
int
ParseNoteFreq(
    char *pszNote,
    int len
    )
{
    int freq = -1;

    TFuncName("ParseNoteFreq");
    TLevel(FUNC);
    TEnterMsg(("Note=%s,len=%d", pszNote, len));

    if ((pszNote[0] >= 'A') && (pszNote[0] <= 'G'))
    {
        int noteNum = pszNote[0] - 'C';
        int i;

        if (noteNum < 0)
        {
            noteNum = (noteNum + 7)*2 + 3;
        }
        else if (noteNum > 2)
        {
            noteNum = noteNum*2 + 3;
        }
        else
        {
            noteNum = noteNum*2 + 4;
        }

        i = 1;
        if (pszNote[i] == '#')
        {
            noteNum++;
            i++;
        }
        else if (pszNote[i] == 'b')
        {
            noteNum--;
            i++;
        }

        if ((pszNote[i] >= '1') && (pszNote[i] <= '8'))
        {
            float f;

            noteNum += (pszNote[i] - '1')*12;
            f = 440*pow(2, (noteNum - 49.0)/12.0);
            freq = (int)round(f);
        }
    }
    else if (pszNote[0] == 'R')
    {
        freq = 0;
    }

    TExitMsg(("=%d", freq));
    return freq;
}   //ParseNoteFreq

/**
 *  This function parses the note duration from the song segment string.
 *
 *  @param pszNote Points to the current note string.
 *  @param len Specifies the current note string length.
 *  @param barDuration Specifies the bar duration.
 */
int
ParseNoteDuration(
    char *pszNote,
    int len,
    int barDuration
    )
{
    int duration = -1;
    float dur = 0.0;
    long data;
    bool hasMore = false;

    TFuncName("ParseNoteFreq");
    TLevel(FUNC);
    TEnterMsg(("Note=%s,len=%d,barDur=%d", pszNote, len, barDuration));

    do
    {
        if (!sscanf(pszNote, "%d", &data))
        {
            dur = -1.0;
            break;
        }
        else
        {
            int n;
            dur += (float)barDuration/data;
            n = StringFind(pszNote, ".");
            if ((n > 0) && (n < len))
            {
                hasMore = true;
            }
            else
            {
                n = len;
                hasMore = false;
            }

            if (pszNote[n - 1] == '+')
            {
                dur += (float)barDuration/data/2;
            }

            if (hasMore)
            {
                pszNote += n + 1;
            }
        }
    } while (hasMore);

    if (dur != -1.0)
    {
        duration = (int)round(dur);
    }

    TExitMsg(("=%d", duration));
    return duration;
}   //ParseNoteDuration

/**
 *  This function parses the note info from the song segment string.
 *
 *  @param pszNote Points to the current note string.
 *  @param barDuration Specifies the bar duration.
 *  @param freq Points to the variable to receive the note frequency.
 *  @param duration Points to the variable to receive the note duration.
 */
int
ParseNote(
    char *pszNote,
    int barDuration,
    int *freq,
    int *duration
    )
{
    int len;
    int n;

    TFuncName("ParseNote");
    TLevel(FUNC);
    TEnterMsg(("Note=%s,barDur=%d", pszNote, barDuration));

    n = StringFind(pszNote, ",");
    if (n == -1)
    {
        //
        // Cannot find ',', could be the last note.
        //
        len = strlen(pszNote);
    }
    else
    {
        len = n + 1;
    }

    n = StringFind(pszNote, ".");
    if ((n <= 0) || (n >= len))
    {
        //
        // Cannot find '.' within the current token.
        //
        len = -1;
    }
    else
    {
        *freq = ParseNoteFreq(pszNote, n);
        TInfo(("%p: %s", pszNote, pszNote));
        pszNote += n + 1;
        if (*freq == -1)
        {
            TErr(("Note not found!"));
            len = -1;
        }
        else
        {
            *duration = ParseNoteDuration(pszNote, len - n - 1, barDuration);
            if (*duration == -1)
            {
                TErr(("Invalid duration!"));
                len = -1;
            }
        }
    }

    TExitMsg(("=%d(f=%d,d=%d)", len, *freq, *duration));
    return len;
}   //ParseNote

/**
 *  This function initializes the song object.
 *
 *  @param song Points to the SONG structure.
 *  @param ppszSong Points to the array of song segment strings.
 */
void
SongInit(
    SONG  &song,
    char **ppszSong
    )
{
    TFuncName("SongInit");
    TLevel(INIT);
    TEnter();

    song.ppszSong = ppszSong;
    song.ppszCurrSeg = ppszSong;
    song.pszNextNote = *ppszSong;
    song.flags = 0;
    song.barDuration = DEF_BAR_DURATION;
    song.options = 0;
    song.sm = NULL;
    song.evtType = 0;

    TExit();
    return;
}   //SongInit

/**
 *  This function is called to resume the song from the last playing note.
 *
 *  @param song Points to the SONG structure.
 */
void
SongResume(
    SONG &song
    )
{
    TFuncName("SongResume");
    TLevel(API);
    TEnter();

    song.flags |= SONGF_STARTED;

    TExit();
    return;
}   //SongResume

/**
 *  This function is called to start playing the song.
 *
 *  @param song Points to the SONG structure.
 *  @param barDuration Specifies the duration of a bar in msec.
 *  @param options Optionally specifies the song options:
 *         SONGO_REPEAT - Repeat playing the song forever.
 *  @param fPause Optionally specifies whether to start the song in
 *         paused state.
 *  @param sm Optionally points to the state machine that needs completion
 *         notification.
 *  @param evtType Optionally specifies the event type to be signaled when
 *         PID operation is completed. Required only if sm is not NULL.
 */
void
SongStart(
    SONG &song,
    int   barDuration = DEF_BAR_DURATION,
    int   options = 0,
    bool  fPause = false,
    SM   *sm = NULL,
    int   evtType = 0
    )
{
    TFuncName("SongStart");
    TLevel(API);
    TEnterMsg(("scale=%d", durationScale));

    song.ppszCurrSeg = song.ppszSong;
    song.pszNextNote = *song.ppszSong;
    song.barDuration = barDuration;
    song.options = options;
    song.sm = sm;
    song.evtType = evtType;
    if (!fPause)
    {
        SongResume(song);
    }

    TExit();
    return;
}   //SongStart

/**
 *  This function is called to stop playing the song.
 *
 *  @param song Points to the SONG structure.
 */
void
SongStop(
    SONG &song
    )
{
    TFuncName("SongStop");
    TLevel(API);
    TEnter();

    song.flags &= ~SONGF_STARTED;

    TExit();
    return;
}   //SongStop

/**
 *  This function is called periodically to play the notes of the song.
 *
 *  @param song Points to the SONG structure.
 */
void
SongTask(
    SONG &song
    )
{
    TFuncName("SongTask");
    TLevel(TASK);
    TEnter();

    unsigned long currTime = nPgmTime;

    if ((song.flags & SONGF_STARTED) && !bSoundActive)
    {
        if (*song.pszNextNote == '\0')
        {
            //
            // We have reached the end of the segment.
            //
            song.ppszCurrSeg++;
            if (*song.ppszCurrSeg == NULL)
            {
                if (song.options & SONGO_REPEAT)
                {
                    SongStart(song, song.barDuration, song.options);
                }
                else
                {
                    SongStop(song);
                    if (song.sm != NULL)
                    {
                        SMSetEvent(*song.sm, song.evtType);
                    }
                }
            }
            else
            {
                song.pszNextNote = *song.ppszCurrSeg;
            }
        }

        if (song.flags & SONGF_STARTED)
        {
            //
            // Parse the next note and play it.
            //
            int freq, duration;
            int n = ParseNote(song.pszNextNote,
                              song.barDuration,
                              &freq, &duration);

            if (n == -1)
            {
                SongStop(song);
                song.flags |= SONGF_PARSE_ERROR;
            }
            else
            {
                song.pszNextNote += n;
                PlayTone(freq, duration/10);
            }
        }
    }

    TExit();
    return;
}   //SongTask

#endif  //ifndef _SONG_H
