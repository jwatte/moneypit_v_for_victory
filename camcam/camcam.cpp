/* see bottom for original copyright */

// We use some GNU extensions (basename)
#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <memory.h>
#include <sysexits.h>
#include <pthread.h>

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#define VERSION_STRING __DATE__

extern "C" {

#include "bcm_host.h"
#include "interface/vcos/vcos.h"

#include "interface/mmal/mmal.h"
#include "interface/mmal/mmal_logging.h"
#include "interface/mmal/mmal_buffer.h"
#include "interface/mmal/util/mmal_util.h"
#include "interface/mmal/util/mmal_util_params.h"
#include "interface/mmal/util/mmal_default_components.h"
#include "interface/mmal/util/mmal_connection.h"
#include "interface/mmal/mmal_parameters_camera.h"

#include "RaspiCamControl.h"
#include "RaspiPreview.h"
#include "RaspiCLI.h"

#include <semaphore.h>

}

#include <string>

#include "settings.h"
#include "gui.h"
#include "detect.h"
#include "detect_inner.h"
#include "gpiofs.h"
#include "sync.h"


#define INLINE_HEADERS 1
#define SEGMENT_SECONDS 20
//  10 == extreme quality, 40 == high compression
#define QUANT_PARAM 16


// Standard port setting for the camera component
#define MMAL_CAMERA_PREVIEW_PORT 0
#define MMAL_CAMERA_VIDEO_PORT 1
#define MMAL_CAMERA_CAPTURE_PORT 2

// Port configuration for the splitter component
#define SPLITTER_OUTPUT_PORT 0
#define SPLITTER_PREVIEW_PORT 1

// Video format information
// 0 implies variable
#define VIDEO_FRAME_RATE_NUM 60
#define VIDEO_FRAME_RATE_DEN 1

/// Video render needs at least 2 buffers.
#define VIDEO_OUTPUT_BUFFERS_NUM 3

// Max bitrate we allow for recording
const int MAX_BITRATE_MJPEG = 25000000; // 25Mbits/s
const int MAX_BITRATE_LEVEL4 = 25000000; // 25Mbits/s
const int MAX_BITRATE_LEVEL42 = 62500000; // 62.5Mbits/s

/// Interval at which we check for an failure abort during capture
const int ABORT_INTERVAL = 100; // ms




extern "C" int mmal_status_to_int(MMAL_STATUS_T status);
static void signal_handler(int signal_number);

// Forward
typedef struct RASPIVID_STATE_S RASPIVID_STATE;

/** Struct used to pass information in encoder port userdata to callback
*/
typedef struct
{
    FILE *file_handle;                   /// File handle to write buffer data to.
    RASPIVID_STATE *pstate;              /// pointer to our state in case required in callback
    int abort;                           /// Set to 1 in callback if an error occurs to attempt to abort the capture
    FILE *raw_file_handle;               /// File handle to write raw data to.
    int  flush_buffers;
} PORT_USERDATA;

/** Possible raw output formats
*/
typedef enum {
    RAW_OUTPUT_FMT_YUV = 1,
    RAW_OUTPUT_FMT_RGB,
    RAW_OUTPUT_FMT_GRAY,
} RAW_OUTPUT_FMT;

/** Structure containing all state information for the current run
*/
struct RASPIVID_STATE_S
{
    int width;                          /// Requested width of image
    int height;                         /// requested height of image
    MMAL_FOURCC_T encoding;             /// Requested codec video encoding (MJPEG or H264)
    int bitrate;                        /// Requested bitrate
    int framerate;                      /// Requested frame rate (fps)
    int intraperiod;                    /// Intra-refresh period (key frame rate)
    int quantisationParameter;          /// Quantisation parameter - quality. Set bitrate 0 and set this for variable bitrate
    char *filename;                     /// filename of output file
    int verbose;                        /// !0 if want detailed run information
    int immutableInput;                 /// Flag to specify whether encoder works in place or creates a new buffer. Result is preview can display either
    /// the camera output or the encoder output (with compression artifacts)
    int profile;                        /// H264 profile to use for encoding
    int level;                          /// H264 level to use for encoding

    int segmentSize;                    /// Segment mode In timed cycle mode, the amount of time the capture is off per cycle
    int segmentNumber;                  /// Current segment counter
    int shouldRecord;

    RASPIPREVIEW_PARAMETERS preview_parameters;   /// Preview setup parameters
    RASPICAM_CAMERA_PARAMETERS camera_parameters; /// Camera setup parameters

    MMAL_COMPONENT_T *camera_component;    /// Pointer to the camera component
    MMAL_COMPONENT_T *splitter_component;  /// Pointer to the splitter component
    MMAL_COMPONENT_T *encoder_component;   /// Pointer to the encoder component
    MMAL_CONNECTION_T *preview_connection; /// Pointer to the connection from camera or splitter to preview
    MMAL_CONNECTION_T *splitter_connection;/// Pointer to the connection from camera to splitter
    MMAL_CONNECTION_T *encoder_connection; /// Pointer to the connection from camera to encoder

    MMAL_POOL_T *splitter_pool; /// Pointer to the pool of buffers used by splitter output port 0
    MMAL_POOL_T *encoder_pool; /// Pointer to the pool of buffers used by encoder output port

    PORT_USERDATA callback_data;        /// Used to move data to the encoder callback

    RAW_OUTPUT_FMT raw_output_fmt;       /// The raw video format
    char *raw_filename;                  /// Filename for raw video output
    int cameraNum;                       /// Camera number
    int settings;                        /// Request settings from the camera
    int sensor_mode;			            /// Sensor mode. 0=auto. Check docs/forum for modes selected by other values.
    int intra_refresh_type;              /// What intra refresh type to use. -1 to not set.
    int frame;
    int64_t starttime;
    int64_t lasttime;
};


/// Structure to cross reference H264 profile strings against the MMAL parameter equivalent
static XREF_T  profile_map[] =
{
    {"baseline",     MMAL_VIDEO_PROFILE_H264_BASELINE},
    {"main",         MMAL_VIDEO_PROFILE_H264_MAIN},
    {"high",         MMAL_VIDEO_PROFILE_H264_HIGH},
    //   {"constrained",  MMAL_VIDEO_PROFILE_H264_CONSTRAINED_BASELINE} // Does anyone need this?
};

static int profile_map_size = sizeof(profile_map) / sizeof(profile_map[0]);

/// Structure to cross reference H264 level strings against the MMAL parameter equivalent
static XREF_T  level_map[] =
{
    {"4",           MMAL_VIDEO_LEVEL_H264_4},
    {"4.1",         MMAL_VIDEO_LEVEL_H264_41},
    {"4.2",         MMAL_VIDEO_LEVEL_H264_42},
};

static int level_map_size = sizeof(level_map) / sizeof(level_map[0]);

static XREF_T  intra_refresh_map[] =
{
    {"cyclic",       MMAL_VIDEO_INTRA_REFRESH_CYCLIC},
    {"adaptive",     MMAL_VIDEO_INTRA_REFRESH_ADAPTIVE},
    {"both",         MMAL_VIDEO_INTRA_REFRESH_BOTH},
    {"cyclicrows",   MMAL_VIDEO_INTRA_REFRESH_CYCLIC_MROWS},
    //   {"random",       MMAL_VIDEO_INTRA_REFRESH_PSEUDO_RAND} Cannot use random, crashes the encoder. No idea why.
};

static int intra_refresh_map_size = sizeof(intra_refresh_map) / sizeof(intra_refresh_map[0]);

static XREF_T  raw_output_fmt_map[] =
{
    {"yuv",  RAW_OUTPUT_FMT_YUV},
    {"rgb",  RAW_OUTPUT_FMT_RGB},
    {"gray", RAW_OUTPUT_FMT_GRAY},
};

static int raw_output_fmt_map_size = sizeof(raw_output_fmt_map) / sizeof(raw_output_fmt_map[0]);

static void display_valid_parameters(char const *app_name);

/// Command ID's and Structure defining our command line options
#define CommandHelp         0
#define CommandWidth        1
#define CommandHeight       2
#define CommandBitrate      3
#define CommandOutput       4
#define CommandVerbose      5
#define CommandFramerate    8
#define CommandPreviewEnc   9
#define CommandIntraPeriod  10
#define CommandProfile      11
#define CommandInitialState 15
#define CommandQP           16
#define CommandSegmentFile  18
#define CommandSegmentStart 20
#define CommandSplitWait    21
#define CommandCamSelect    24
#define CommandSettings     25
#define CommandSensorMode   26
#define CommandIntraRefreshType 27
#define CommandFlush        28
#define CommandCodec        30
#define CommandLevel        31
#define CommandRawFormat    33

static COMMAND_LIST cmdline_commands[] =
{
    { CommandHelp,          "-help",       "?",  "This help information", 0 },
    { CommandWidth,         "-width",      "w",  "Set image width <size>. Default 1920", 1 },
    { CommandHeight,        "-height",     "h",  "Set image height <size>. Default 1080", 1 },
    { CommandBitrate,       "-bitrate",    "b",  "Set bitrate. Use bits per second (e.g. 10MBits/s would be -b 10000000)", 1 },
    { CommandOutput,        "-output",     "o",  "Output filename <filename> (to write to stdout, use '-o -').\n", 1 },
    { CommandVerbose,       "-verbose",    "v",  "Output verbose information during run", 0 },
    { CommandFramerate,     "-framerate",  "fps","Specify the frames per second to record", 1},
    { CommandPreviewEnc,    "-penc",       "e",  "Display preview image *after* encoding (shows compression artifacts)", 0},
    { CommandIntraPeriod,   "-intra",      "g",  "Specify the intra refresh period (key frame rate/GoP size). Zero to produce an initial I-frame and then just P-frames.", 1},
    { CommandProfile,       "-profile",    "pf", "Specify H264 profile to use for encoding", 1},
    { CommandInitialState,  "-initial",    "i",  "Initial state. Use 'record' or 'pause'. Default 'record'", 1},
    { CommandQP,            "-qp",         "qp", "Quantisation parameter. Use approximately 10-40. Default 0 (off)", 1},
    { CommandSegmentFile,   "-segment",    "sg", "Segment output file in to multiple files at specified interval <ms>", 1},
    { CommandSegmentStart,  "-start",      "sn", "In segment mode, start with specified segment number", 1},
    { CommandSplitWait,     "-split",      "sp", "In wait mode, create new output file for each start event", 0},
    { CommandCamSelect,     "-camselect",  "cs", "Select camera <number>. Default 0", 1 },
    { CommandSettings,      "-settings",   "set","Retrieve camera settings and write to stdout", 0},
    { CommandSensorMode,    "-mode",       "md", "Force sensor mode. 0=auto. See docs for other modes available", 1},
    { CommandIntraRefreshType,"-irefresh", "if", "Set intra refresh type", 1},
    { CommandFlush,         "-flush",      "fl",  "Flush buffers in order to decrease latency", 0 },
    { CommandCodec,         "-codec",      "cd", "Specify the codec to use - H264 (default) or MJPEG", 1 },
    { CommandLevel,         "-level",      "lev","Specify H264 level to use for encoding", 1},
    { CommandRawFormat,     "-raw-format", "rf", "Specify output format for raw video. Default is yuv", 1},
};

static int cmdline_commands_size = sizeof(cmdline_commands) / sizeof(cmdline_commands[0]);



/**
 * Assign a default set of parameters to the state passed in
 *
 * @param state Pointer to state structure to assign defaults to
 */
static void default_status(RASPIVID_STATE *state)
{
    if (!state)
    {
        vcos_assert(0);
        return;
    }

    // Default everything to zero
    memset(state, 0, sizeof(RASPIVID_STATE));

    // Now set anything non-zero
    state->width = 1920;       // Default to 1080p
    state->height = 1080;
    state->encoding = MMAL_ENCODING_H264;
    state->bitrate = 17000000; // This is a decent default bitrate for 1080p
    state->framerate = VIDEO_FRAME_RATE_NUM;
    state->intraperiod = -1;    // Not set
    state->quantisationParameter = 0;
    state->immutableInput = 1;
    state->profile = MMAL_VIDEO_PROFILE_H264_HIGH;
    state->level = MMAL_VIDEO_LEVEL_H264_4;

    state->segmentSize = 0; // don't segment
    state->segmentNumber = 1;
    state->shouldRecord = 0;

    state->cameraNum = 0;
    state->settings = 0;
    state->sensor_mode = 7;  //  640x480, full FOV, 4x4 binning, high FPS, on v1 camera

    state->intra_refresh_type = -1;

    state->frame = 0;


    // Setup preview window defaults
    raspipreview_set_defaults(&state->preview_parameters);

    // Set up the camera_parameters to default
    raspicamcontrol_set_defaults(&state->camera_parameters);
}


/**
 * Dump image state parameters to stderr.
 *
 * @param state Pointer to state structure to assign defaults to
 */
static void dump_status(RASPIVID_STATE *state)
{
    if (!state)
    {
        vcos_assert(0);
        return;
    }

    fprintf(stderr, "Width %d, Height %d, filename %s\n", state->width, state->height, state->filename);
    fprintf(stderr, "bitrate %d, framerate %d\n", state->bitrate, state->framerate);
    fprintf(stderr, "H264 Profile %s\n", raspicli_unmap_xref(state->profile, profile_map, profile_map_size));
    fprintf(stderr, "H264 Level %s\n", raspicli_unmap_xref(state->level, level_map, level_map_size));
    fprintf(stderr, "H264 Quantisation level %d, Inline headers %s\n", state->quantisationParameter, INLINE_HEADERS ? "Yes" : "No");
    fprintf(stderr, "H264 Intra refresh type %s, period %d\n", raspicli_unmap_xref(state->intra_refresh_type, intra_refresh_map, intra_refresh_map_size), state->intraperiod);

    // Not going to display segment data unless asked for it.
    if (state->segmentSize)
        fprintf(stderr, "Segment size %d ms, initial segment number %d\n", state->segmentSize, state->segmentNumber);

    fprintf(stderr, "Raw output enabled, format %s\n", raspicli_unmap_xref(state->raw_output_fmt, raw_output_fmt_map, raw_output_fmt_map_size));

    fprintf(stderr, "\n\n");

    raspipreview_dump_parameters(&state->preview_parameters);
    raspicamcontrol_dump_parameters(&state->camera_parameters);
}

/**
 * Parse the incoming command line and put resulting parameters in to the state
 *
 * @param argc Number of arguments in command line
 * @param argv Array of pointers to strings from command line
 * @param state Pointer to state structure to assign any discovered parameters to
 * @return Non-0 if failed for some reason, 0 otherwise
 */
static int parse_cmdline(int argc, const char **argv, RASPIVID_STATE *state)
{
    // Parse the command line arguments.
    // We are looking for --<something> or -<abbreviation of something>

    int valid = 1;
    int i;

    for (i = 1; i < argc && valid; i++)
    {
        int command_id, num_parameters;

        if (!argv[i])
            continue;

        if (argv[i][0] != '-')
        {
            valid = 0;
            continue;
        }

        // Assume parameter is valid until proven otherwise
        valid = 1;

        command_id = raspicli_get_command_id(cmdline_commands, cmdline_commands_size, &argv[i][1], &num_parameters);

        // If we found a command but are missing a parameter, continue (and we will drop out of the loop)
        if (command_id != -1 && num_parameters > 0 && (i + 1 >= argc) )
            continue;

        //  We are now dealing with a command line option
        switch (command_id)
        {
            case CommandHelp:
                display_valid_parameters(basename(argv[0]));
                return -1;

            case CommandWidth: // Width > 0
                if (sscanf(argv[i + 1], "%u", &state->width) != 1)
                    valid = 0;
                else
                    i++;
                break;

            case CommandHeight: // Height > 0
                if (sscanf(argv[i + 1], "%u", &state->height) != 1)
                    valid = 0;
                else
                    i++;
                break;

            case CommandBitrate: // 1-100
                if (sscanf(argv[i + 1], "%u", &state->bitrate) == 1)
                {
                    i++;
                }
                else
                    valid = 0;

                break;

            case CommandOutput:  // output filename
                {
                    int len = strlen(argv[i + 1]);
                    if (len)
                    {
                        state->filename = (char *)malloc(len + 1);
                        vcos_assert(state->filename);
                        if (state->filename)
                            strncpy(state->filename, argv[i + 1], len+1);
                        i++;
                    }
                    else
                        valid = 0;
                    break;
                }

            case CommandVerbose: // display lots of data during run
                state->verbose = 1;
                break;

            case CommandFramerate: // fps to record
                {
                    if (sscanf(argv[i + 1], "%u", &state->framerate) == 1)
                    {
                        // TODO : What limits do we need for fps 1 - 30 - 120??
                        i++;
                    }
                    else
                        valid = 0;
                    break;
                }

            case CommandPreviewEnc:
                state->immutableInput = 0;
                break;

            case CommandIntraPeriod: // key frame rate
                {
                    if (sscanf(argv[i + 1], "%u", &state->intraperiod) == 1)
                        i++;
                    else
                        valid = 0;
                    break;
                }

            case CommandQP: // quantisation parameter
                {
                    if (sscanf(argv[i + 1], "%u", &state->quantisationParameter) == 1)
                        i++;
                    else
                        valid = 0;
                    break;
                }

            case CommandProfile: // H264 profile
                {
                    state->profile = raspicli_map_xref(argv[i + 1], profile_map, profile_map_size);

                    if( state->profile == -1)
                        state->profile = MMAL_VIDEO_PROFILE_H264_HIGH;

                    i++;
                    break;
                }

            case CommandSegmentFile: // Segment file in to chunks of specified time
                {
                    if (sscanf(argv[i + 1], "%u", &state->segmentSize) == 1)
                    {
                        // Must enable inline headers for this to work
                        i++;
                    }
                    else
                        valid = 0;
                    break;
                }

            case CommandSegmentStart: // initial segment number
                {
                    if (sscanf(argv[i + 1], "%u", &state->segmentNumber) == 1)
                        i++;
                    else
                        valid = 0;
                    break;
                }

            case CommandCamSelect:  //Select camera input port
                {
                    if (sscanf(argv[i + 1], "%u", &state->cameraNum) == 1)
                    {
                        i++;
                    }
                    else
                        valid = 0;
                    break;
                }

            case CommandSettings:
                state->settings = 1;
                break;

            case CommandSensorMode:
                {
                    if (sscanf(argv[i + 1], "%u", &state->sensor_mode) == 1)
                    {
                        i++;
                    }
                    else
                        valid = 0;
                    break;
                }

            case CommandIntraRefreshType:
                {
                    state->intra_refresh_type = raspicli_map_xref(argv[i + 1], intra_refresh_map, intra_refresh_map_size);
                    i++;
                    break;
                }

            case CommandFlush:
                {
                    state->callback_data.flush_buffers = 1;
                    break;
                }
            case CommandCodec:  // codec type
                {
                    int len = strlen(argv[i + 1]);
                    if (len)
                    {
                        if (len==4 && !strncmp("H264", argv[i+1], 4))
                            state->encoding = MMAL_ENCODING_H264;
                        else  if (len==5 && !strncmp("MJPEG", argv[i+1], 5))
                            state->encoding = MMAL_ENCODING_MJPEG;
                        else
                            valid = 0;
                        i++;
                    }
                    else
                        valid = 0;
                    break;
                }

            case CommandLevel: // H264 level
                {
                    state->level = raspicli_map_xref(argv[i + 1], level_map, level_map_size);

                    if( state->level == -1)
                        state->level = MMAL_VIDEO_LEVEL_H264_4;

                    i++;
                    break;
                }

            case CommandRawFormat:
                {
                    state->raw_output_fmt = (RAW_OUTPUT_FMT)raspicli_map_xref(argv[i + 1], raw_output_fmt_map, raw_output_fmt_map_size);

                    if (state->raw_output_fmt == -1)
                        valid = 0;

                    i++;
                    break;
                }

            default:
                {
                    // Try parsing for any image specific parameters
                    // result indicates how many parameters were used up, 0,1,2
                    // but we adjust by -1 as we have used one already
                    const char *second_arg = (i + 1 < argc) ? argv[i + 1] : NULL;
                    int parms_used = (raspicamcontrol_parse_cmdline(&state->camera_parameters, &argv[i][1], second_arg));

                    // Still unused, try preview options
                    if (!parms_used)
                        parms_used = raspipreview_parse_cmdline(&state->preview_parameters, &argv[i][1], second_arg);


                    // If no parms were used, this must be a bad parameters
                    if (!parms_used)
                        valid = 0;
                    else
                        i += parms_used - 1;

                    break;
                }
        }
    }

    if (!valid)
    {
        fprintf(stderr, "Invalid command line option (%s)\n", argv[i-1]);
        return 1;
    }

    // Always disable verbose if output going to stdout
    if (state->filename && state->filename[0] == '-')
    {
        state->verbose = 0;
    }

    return 0;
}

/**
 * Display usage information for the application to stdout
 *
 * @param app_name String to display as the application name
 */
static void display_valid_parameters(char const *app_name)
{
    int i;

    fprintf(stdout, "Display camera output to display, and optionally saves an H264 capture at requested bitrate\n\n");
    fprintf(stdout, "\nusage: %s [options]\n\n", app_name);

    fprintf(stdout, "Image parameter commands\n\n");

    raspicli_display_help(cmdline_commands, cmdline_commands_size);

    // Profile options
    fprintf(stdout, "\n\nH264 Profile options :\n%s", profile_map[0].mode );

    for (i=1;i<profile_map_size;i++)
    {
        fprintf(stdout, ",%s", profile_map[i].mode);
    }

    // Level options
    fprintf(stdout, "\n\nH264 Level options :\n%s", level_map[0].mode );

    for (i=1;i<level_map_size;i++)
    {
        fprintf(stdout, ",%s", level_map[i].mode);
    }

    // Intra refresh options
    fprintf(stdout, "\n\nH264 Intra refresh options :\n%s", intra_refresh_map[0].mode );

    for (i=1;i<intra_refresh_map_size;i++)
    {
        fprintf(stdout, ",%s", intra_refresh_map[i].mode);
    }

    // Raw output format options
    fprintf(stdout, "\n\nRaw output format options :\n%s", raw_output_fmt_map[0].mode );

    for (i=1;i<raw_output_fmt_map_size;i++)
    {
        fprintf(stdout, ",%s", raw_output_fmt_map[i].mode);
    }

    fprintf(stdout, "\n");

    // Help for preview options
    raspipreview_display_help();

    // Now display any help information from the camcontrol code
    raspicamcontrol_display_help();

    fprintf(stdout, "\n");

    return;
}

// Our main data storage vessel..
RASPIVID_STATE state;


/**
 *  buffer header callback function for camera control
 *
 *  Callback will dump buffer data to the specific file
 *
 * @param port Pointer to port from which callback originated
 * @param buffer mmal buffer header pointer
 */
static void camera_control_callback(MMAL_PORT_T *port, MMAL_BUFFER_HEADER_T *buffer)
{
    if (buffer->cmd == MMAL_EVENT_PARAMETER_CHANGED)
    {
        MMAL_EVENT_PARAMETER_CHANGED_T *param = (MMAL_EVENT_PARAMETER_CHANGED_T *)buffer->data;
        switch (param->hdr.id) {
            case MMAL_PARAMETER_CAMERA_SETTINGS:
                {
                    MMAL_PARAMETER_CAMERA_SETTINGS_T *settings = (MMAL_PARAMETER_CAMERA_SETTINGS_T*)param;
                    vcos_log_error("Exposure now %u, analog gain %u/%u, digital gain %u/%u",
                            settings->exposure,
                            settings->analog_gain.num, settings->analog_gain.den,
                            settings->digital_gain.num, settings->digital_gain.den);
                    vcos_log_error("AWB R=%u/%u, B=%u/%u",
                            settings->awb_red_gain.num, settings->awb_red_gain.den,
                            settings->awb_blue_gain.num, settings->awb_blue_gain.den
                            );
                }
                break;
        }
    }
    else if (buffer->cmd == MMAL_EVENT_ERROR)
    {
        vcos_log_error("No data received from sensor. Check all connections, including the Sunny one on the camera board");
    }
    else
    {
        vcos_log_error("Received unexpected camera control callback event, 0x%08x", buffer->cmd);
    }

    mmal_buffer_header_release(buffer);
}


/**
 * Open a file based on the settings in state
 *
 * @param state Pointer to state
 */
static FILE *open_filename(RASPIVID_STATE *pState, char *filename)
{
    FILE *new_handle = NULL;
    char *tempname = NULL;

    char fnpat[1024];
    // Create a new filename string
    strncpy(fnpat, filename, sizeof(fnpat));
    fnpat[sizeof(fnpat)-20] = 0;
    char *dot = strrchr(fnpat, '.');
    if (dot) {
        *dot = 0;
        dot = strrchr(filename, '.');
    }
    if (!dot || strlen(dot) > 10) {
        dot = (char *)".h264";
    }
    char *p;
    while ((p = strchr(fnpat, '%')) != NULL) {
        *p = '_';
    }
    strcat(fnpat, "-%04d");
    strcat(fnpat, dot);
    asprintf(&tempname, fnpat, pState->segmentNumber);

    if (pState->verbose)
    {
        fprintf(stderr, "Opening output file \"%s\"\n", tempname);
    }

    new_handle = fopen(tempname, "wb");
    if (!new_handle) {
        perror(tempname);
    } else {
        pState->segmentNumber++;
    }

    free(tempname);

    return new_handle;
}


uint64_t get_microseconds() {
    return vcos_getmicrosecs64();
}


static int64_t base_time =  -1;

/**
 *  buffer header callback function for encoder
 *
 *  Callback will dump buffer data to the specific file
 *
 * @param port Point
 * @param buffer mmal buffer header pointer
 */
static void encoder_buffer_callback(MMAL_PORT_T *port, MMAL_BUFFER_HEADER_T *buffer)
{
    MMAL_BUFFER_HEADER_T *new_buffer;
    // All our segment times based on the receipt of the first encoder callback
    if (base_time == -1)
        base_time = vcos_getmicrosecs64()/1000;

    // We pass our file handle and other stuff in via the userdata field.

    PORT_USERDATA *pData = (PORT_USERDATA *)port->userdata;
    if (pData != NULL) {
        RASPIVID_STATE *pstate = pData->pstate;
        int64_t current_time = vcos_getmicrosecs64()/1000;

        int bytes_written = buffer->length;

        if (pstate->shouldRecord) {
            if (buffer->flags & MMAL_BUFFER_HEADER_FLAG_CONFIG) {
                if (pData->pstate->segmentSize && (current_time > base_time + pData->pstate->segmentSize)) {
                    if (pData->file_handle) {
                        fclose(pData->file_handle);
                        pData->file_handle = NULL;
                    }
                    base_time = current_time;
                }
                if (!pData->file_handle) {
                    pData->file_handle = open_filename(pstate, pstate->filename);
                }
            }
            if (pData->file_handle != NULL && buffer->length)
            {
                mmal_buffer_header_mem_lock(buffer);
                if(buffer->flags & MMAL_BUFFER_HEADER_FLAG_CODECSIDEINFO)
                {
                    //We do not want to save inlineMotionVectors...
                    bytes_written = buffer->length;
                }
                else
                {
                    bytes_written = fwrite(buffer->data, 1, buffer->length, pData->file_handle);
                    if(pData->flush_buffers) fflush(pData->file_handle);
                }

                mmal_buffer_header_mem_unlock(buffer);

                if (bytes_written != (int)buffer->length)
                {
                    vcos_log_error("Failed to write buffer data (%d from %d)- aborting", bytes_written, buffer->length);
                    pstate->shouldRecord = false;
                    fclose(pData->file_handle);
                    pData->file_handle = NULL;
                }
            }
        } else if (pData->file_handle) {
            fclose(pData->file_handle);
            pData->file_handle = NULL;
        }
    }

    // release buffer back to the pool
    mmal_buffer_header_release(buffer);

    // and send one back to the port (if still open)
    if (port->is_enabled)
    {
        MMAL_STATUS_T status;

        new_buffer = mmal_queue_get(pData->pstate->encoder_pool->queue);

        if (new_buffer)
            status = mmal_port_send_buffer(port, new_buffer);

        if (!new_buffer || status != MMAL_SUCCESS)
            vcos_log_error("Unable to return a buffer to the encoder port");
    }
}


void recycle_buffer(MMAL_PORT_T *port, MMAL_BUFFER_HEADER_T *buffer) {

    PORT_USERDATA *pData = (PORT_USERDATA *)port->userdata;

    // release buffer back to the pool
    mmal_buffer_header_release(buffer);

    // and send one back to the port (if still open)
    if (pData && port->is_enabled)
    {
        MMAL_STATUS_T status;
        MMAL_BUFFER_HEADER_T *new_buffer = NULL;

        new_buffer = mmal_queue_get(pData->pstate->splitter_pool->queue);

        if (new_buffer)
            status = mmal_port_send_buffer(port, new_buffer);

        if (!new_buffer || status != MMAL_SUCCESS)
            vcos_log_error("Unable to return a buffer to the splitter port");
    }
}

/**
 *  buffer header callback function for splitter
 *
 *  Callback will dump buffer data to the specific file
 *
 * @param port Pointer to port from which callback originated
 * @param buffer mmal buffer header pointer
 */
static void splitter_buffer_callback(MMAL_PORT_T *port, MMAL_BUFFER_HEADER_T *buffer)
{
    PORT_USERDATA *pData = (PORT_USERDATA *)port->userdata;

    bool should_release = true;

    if (pData)
    {
        if (buffer->length)
        {
            should_release = !analyzer_consume(port, buffer);
        }
    }

    if (should_release) {
        recycle_buffer(port, buffer);
    }
}

/**
 * Create the camera component, set up its ports
 *
 * @param state Pointer to state control struct
 *
 * @return MMAL_SUCCESS if all OK, something else otherwise
 *
 */
static MMAL_STATUS_T create_camera_component(RASPIVID_STATE *state)
{
    MMAL_COMPONENT_T *camera = 0;
    MMAL_ES_FORMAT_T *format;
    MMAL_PORT_T *preview_port = NULL, *video_port = NULL, *still_port = NULL;
    int status;
    MMAL_PARAMETER_INT32_T camera_num =
    {{MMAL_PARAMETER_CAMERA_NUM, sizeof(camera_num)}, state->cameraNum};


    /* Create the component */
    status = mmal_component_create(MMAL_COMPONENT_DEFAULT_CAMERA, &camera) ? MMAL_EINVAL : MMAL_SUCCESS;

    if (status != MMAL_SUCCESS)
    {
        vcos_log_error("Failed to create camera component");
        goto error;
    }

    //status = raspicamcontrol_set_all_parameters(camera, &state->camera_parameters);
    status = 0;

    status += raspicamcontrol_set_stereo_mode(camera->output[0], &state->camera_parameters.stereo_mode);
    status += raspicamcontrol_set_stereo_mode(camera->output[1], &state->camera_parameters.stereo_mode);
    status += raspicamcontrol_set_stereo_mode(camera->output[2], &state->camera_parameters.stereo_mode);

    if (status != MMAL_SUCCESS)
    {
        vcos_log_error("Could not set stereo mode : error %d", status);
        goto error;
    }

    status = mmal_port_parameter_set(camera->control, &camera_num.hdr);

    if (status != MMAL_SUCCESS)
    {
        vcos_log_error("Could not select camera : error %d", status);
        goto error;
    }

    if (!camera->output_num)
    {
        status = MMAL_ENOSYS;
        vcos_log_error("Camera doesn't have output ports");
        goto error;
    }

    status = mmal_port_parameter_set_uint32(camera->control, MMAL_PARAMETER_CAMERA_CUSTOM_SENSOR_CONFIG, state->sensor_mode);

    if (status != MMAL_SUCCESS)
    {
        vcos_log_error("Could not set sensor mode : error %d", status);
        goto error;
    }

    preview_port = camera->output[MMAL_CAMERA_PREVIEW_PORT];
    video_port = camera->output[MMAL_CAMERA_VIDEO_PORT];
    still_port = camera->output[MMAL_CAMERA_CAPTURE_PORT];

    if (state->settings)
    {
        MMAL_PARAMETER_CHANGE_EVENT_REQUEST_T change_event_request =
        {{MMAL_PARAMETER_CHANGE_EVENT_REQUEST, sizeof(MMAL_PARAMETER_CHANGE_EVENT_REQUEST_T)},
            MMAL_PARAMETER_CAMERA_SETTINGS, 1};

        status = mmal_port_parameter_set(camera->control, &change_event_request.hdr);
        if ( status != MMAL_SUCCESS )
        {
            vcos_log_error("No camera settings events");
        }
    }

    // Enable the camera, and tell it its control callback function
    status = mmal_port_enable(camera->control, camera_control_callback);

    if (status != MMAL_SUCCESS)
    {
        vcos_log_error("Unable to enable control port : error %d", status);
        goto error;
    }

    //  set up the camera configuration
    {
        MMAL_PARAMETER_CAMERA_CONFIG_T cam_config =
        {
            { MMAL_PARAMETER_CAMERA_CONFIG, sizeof(cam_config) },
            .max_stills_w = (uint32_t)state->width,
            .max_stills_h = (uint32_t)state->height,
            .stills_yuv422 = 0,
            .one_shot_stills = 0,
            .max_preview_video_w = (uint32_t)state->width,
            .max_preview_video_h = (uint32_t)state->height,
            .num_preview_video_frames = (uint32_t)(4 + vcos_max(0, (state->framerate-30)/10)),
            .stills_capture_circular_buffer_height = 0,
            .fast_preview_resume = 0,
            .use_stc_timestamp = MMAL_PARAM_TIMESTAMP_MODE_RAW_STC
        };
        mmal_port_parameter_set(camera->control, &cam_config.hdr);
    }

    // Now set up the port formats

    // Set the encode format on the Preview port
    // HW limitations mean we need the preview to be the same size as the required recorded output

    format = preview_port->format;

    //format->encoding = MMAL_ENCODING_RGB24;
    //format->encoding_variant = 0;
    format->encoding = MMAL_ENCODING_OPAQUE;
    format->encoding_variant = MMAL_ENCODING_I420;

    if(state->camera_parameters.shutter_speed > 6000000)
    {
        MMAL_PARAMETER_FPS_RANGE_T fps_range = {{MMAL_PARAMETER_FPS_RANGE, sizeof(fps_range)},
            { 50, 1000 }, {166, 1000}};
        mmal_port_parameter_set(preview_port, &fps_range.hdr);
    }
    else if(state->camera_parameters.shutter_speed > 1000000)
    {
        MMAL_PARAMETER_FPS_RANGE_T fps_range = {{MMAL_PARAMETER_FPS_RANGE, sizeof(fps_range)},
            { 166, 1000 }, {999, 1000}};
        mmal_port_parameter_set(preview_port, &fps_range.hdr);
    }

    //enable dynamic framerate if necessary
    if (state->camera_parameters.shutter_speed)
    {
        if (state->framerate > 1000000./state->camera_parameters.shutter_speed)
        {
            state->framerate=0;
            if (state->verbose)
                fprintf(stderr, "Enable dynamic frame rate to fulfil shutter speed requirement\n");
        }
    }

    format->encoding = MMAL_ENCODING_OPAQUE;
    format->es->video.width = VCOS_ALIGN_UP(state->width, 32);
    format->es->video.height = VCOS_ALIGN_UP(state->height, 16);
    format->es->video.crop.x = 0;
    format->es->video.crop.y = 0;
    format->es->video.crop.width = state->width;
    format->es->video.crop.height = state->height;
    format->es->video.frame_rate.num = PREVIEW_FRAME_RATE_NUM;
    format->es->video.frame_rate.den = PREVIEW_FRAME_RATE_DEN;

    status = mmal_port_format_commit(preview_port);

    if (status != MMAL_SUCCESS)
    {
        vcos_log_error("camera viewfinder format couldn't be set");
        goto error;
    }

    // Set the encode format on the video  port

    format = video_port->format;
    //format->encoding = MMAL_ENCODING_RGB24;
    //format->encoding_variant = 0;
    format->encoding = MMAL_ENCODING_OPAQUE;
    format->encoding_variant = MMAL_ENCODING_I420;

    if(state->camera_parameters.shutter_speed > 6000000)
    {
        MMAL_PARAMETER_FPS_RANGE_T fps_range = {{MMAL_PARAMETER_FPS_RANGE, sizeof(fps_range)},
            { 50, 1000 }, {166, 1000}};
        mmal_port_parameter_set(video_port, &fps_range.hdr);
    }
    else if(state->camera_parameters.shutter_speed > 1000000)
    {
        MMAL_PARAMETER_FPS_RANGE_T fps_range = {{MMAL_PARAMETER_FPS_RANGE, sizeof(fps_range)},
            { 167, 1000 }, {999, 1000}};
        mmal_port_parameter_set(video_port, &fps_range.hdr);
    }

    format->encoding = MMAL_ENCODING_OPAQUE;
    format->es->video.width = VCOS_ALIGN_UP(state->width, 32);
    format->es->video.height = VCOS_ALIGN_UP(state->height, 16);
    format->es->video.crop.x = 0;
    format->es->video.crop.y = 0;
    format->es->video.crop.width = state->width;
    format->es->video.crop.height = state->height;
    format->es->video.frame_rate.num = state->framerate;
    format->es->video.frame_rate.den = VIDEO_FRAME_RATE_DEN;

    status = mmal_port_format_commit(video_port);

    if (status != MMAL_SUCCESS)
    {
        vcos_log_error("camera video format couldn't be set");
        goto error;
    }

    // Ensure there are enough buffers to avoid dropping frames
    if (video_port->buffer_num < VIDEO_OUTPUT_BUFFERS_NUM)
        video_port->buffer_num = VIDEO_OUTPUT_BUFFERS_NUM;


    // Set the encode format on the still  port

    format = still_port->format;

    format->encoding = MMAL_ENCODING_RGB24;
    format->encoding_variant = 0;
    // format->encoding = MMAL_ENCODING_OPAQUE;
    // format->encoding_variant = MMAL_ENCODING_I420;

    format->es->video.width = VCOS_ALIGN_UP(state->width, 32);
    format->es->video.height = VCOS_ALIGN_UP(state->height, 16);
    format->es->video.crop.x = 0;
    format->es->video.crop.y = 0;
    format->es->video.crop.width = state->width;
    format->es->video.crop.height = state->height;
    format->es->video.frame_rate.num = 0;
    format->es->video.frame_rate.den = 1;

    status = mmal_port_format_commit(still_port);

    if (status != MMAL_SUCCESS)
    {
        vcos_log_error("camera still format couldn't be set");
        goto error;
    }

    /* Ensure there are enough buffers to avoid dropping frames */
    if (still_port->buffer_num < VIDEO_OUTPUT_BUFFERS_NUM)
        still_port->buffer_num = VIDEO_OUTPUT_BUFFERS_NUM;

    /* Enable component */
    status = mmal_component_enable(camera);

    if (status != MMAL_SUCCESS)
    {
        vcos_log_error("camera component couldn't be enabled");
        goto error;
    }

    // Note: this sets lots of parameters that were not individually addressed before.
    raspicamcontrol_set_all_parameters(camera, &state->camera_parameters);

    state->camera_component = camera;

    if (state->verbose)
        fprintf(stderr, "Camera component done\n");

    return (MMAL_STATUS_T)status;

error:

    if (camera)
        mmal_component_destroy(camera);

    return (MMAL_STATUS_T)status;
}

/**
 * Destroy the camera component
 *
 * @param state Pointer to state control struct
 *
 */
static void destroy_camera_component(RASPIVID_STATE *state)
{
    if (state->camera_component)
    {
        mmal_component_destroy(state->camera_component);
        state->camera_component = NULL;
    }
}

/**
 * Create the splitter component, set up its ports
 *
 * @param state Pointer to state control struct
 *
 * @return MMAL_SUCCESS if all OK, something else otherwise
 *
 */
static MMAL_STATUS_T create_splitter_component(RASPIVID_STATE *state)
{
    MMAL_COMPONENT_T *splitter = 0;
    MMAL_PORT_T *splitter_output = NULL;
    MMAL_ES_FORMAT_T *format;
    MMAL_STATUS_T status;
    MMAL_POOL_T *pool;
    int i;

    if (state->camera_component == NULL)
    {
        status = MMAL_ENOSYS;
        vcos_log_error("Camera component must be created before splitter");
        goto error;
    }

    /* Create the component */
    status = mmal_component_create(MMAL_COMPONENT_DEFAULT_VIDEO_SPLITTER, &splitter);

    if (status != MMAL_SUCCESS)
    {
        vcos_log_error("Failed to create splitter component");
        goto error;
    }

    if (!splitter->input_num)
    {
        status = MMAL_ENOSYS;
        vcos_log_error("Splitter doesn't have any input port");
        goto error;
    }

    if (splitter->output_num < 2)
    {
        status = MMAL_ENOSYS;
        vcos_log_error("Splitter doesn't have enough output ports");
        goto error;
    }

    /* Ensure there are enough buffers to avoid dropping frames: */
    mmal_format_copy(splitter->input[0]->format, state->camera_component->output[MMAL_CAMERA_PREVIEW_PORT]->format);

    if (splitter->input[0]->buffer_num < VIDEO_OUTPUT_BUFFERS_NUM)
        splitter->input[0]->buffer_num = VIDEO_OUTPUT_BUFFERS_NUM;

    status = mmal_port_format_commit(splitter->input[0]);

    if (status != MMAL_SUCCESS)
    {
        vcos_log_error("Unable to set format on splitter input port");
        goto error;
    }

    /* Splitter can do format conversions, configure format for its output port: */
    for (i = 0; i < (int)splitter->output_num; i++)
    {
        mmal_format_copy(splitter->output[i]->format, splitter->input[0]->format);

        if (i == SPLITTER_OUTPUT_PORT)
        {
            format = splitter->output[i]->format;

            switch (state->raw_output_fmt)
            {
                case RAW_OUTPUT_FMT_YUV:
                case RAW_OUTPUT_FMT_GRAY: /* Grayscale image contains only luma (Y) component */
                    fprintf(stderr, "split format: YUV\n");
                    format->encoding = MMAL_ENCODING_I420;
                    format->encoding_variant = MMAL_ENCODING_I420;
                    break;
                case RAW_OUTPUT_FMT_RGB:
                    fprintf(stderr, "split format: RGB\n");
                    if (mmal_util_rgb_order_fixed(state->camera_component->output[MMAL_CAMERA_CAPTURE_PORT])) {
                        fprintf(stderr, "new RGB order\n");
                        format->encoding = MMAL_ENCODING_RGB24;
                    }
                    else {
                        fprintf(stderr, "old BGR order\n");
                        format->encoding = MMAL_ENCODING_BGR24;
                    }
                    format->encoding_variant = 0;  /* Irrelevant when not in opaque mode */
                    break;
                default:
                    status = MMAL_EINVAL;
                    vcos_log_error("unknown raw output format");
                    goto error;
            }
        }

        status = mmal_port_format_commit(splitter->output[i]);

        if (status != MMAL_SUCCESS)
        {
            vcos_log_error("Unable to set format on splitter output port %d", i);
            goto error;
        }
    }

    /* Enable component */
    status = mmal_component_enable(splitter);

    if (status != MMAL_SUCCESS)
    {
        vcos_log_error("splitter component couldn't be enabled");
        goto error;
    }

    /* Create pool of buffer headers for the output port to consume */
    splitter_output = splitter->output[SPLITTER_OUTPUT_PORT];
    /*
       if (splitter_output->buffer_size < PROC_WIDTH * PROC_HEIGHT * 3) {
       fprintf(stderr, "splitter_output buffer size is %d should be %d\n",
       splitter_output->buffer_size, PROC_WIDTH * PROC_HEIGHT * 3);
       }
       */
    pool = mmal_port_pool_create(splitter_output, splitter_output->buffer_num, splitter_output->buffer_size);

    if (!pool)
    {
        vcos_log_error("Failed to create buffer header pool for splitter output port %s", splitter_output->name);
    }

    state->splitter_pool = pool;
    state->splitter_component = splitter;

    if (state->verbose)
        fprintf(stderr, "Splitter component done\n");

    return status;

error:

    if (splitter)
        mmal_component_destroy(splitter);

    return status;
}

/**
 * Destroy the splitter component
 *
 * @param state Pointer to state control struct
 *
 */
static void destroy_splitter_component(RASPIVID_STATE *state)
{
    // Get rid of any port buffers first
    if (state->splitter_pool)
    {
        mmal_port_pool_destroy(state->splitter_component->output[SPLITTER_OUTPUT_PORT], state->splitter_pool);
    }

    if (state->splitter_component)
    {
        mmal_component_destroy(state->splitter_component);
        state->splitter_component = NULL;
    }
}

/**
 * Create the encoder component, set up its ports
 *
 * @param state Pointer to state control struct
 *
 * @return MMAL_SUCCESS if all OK, something else otherwise
 *
 */
static MMAL_STATUS_T create_encoder_component(RASPIVID_STATE *state)
{
    MMAL_COMPONENT_T *encoder = 0;
    MMAL_PORT_T *encoder_input = NULL, *encoder_output = NULL;
    MMAL_STATUS_T status;
    MMAL_POOL_T *pool;

    status = mmal_component_create(MMAL_COMPONENT_DEFAULT_VIDEO_ENCODER, &encoder);

    if (status != MMAL_SUCCESS)
    {
        vcos_log_error("Unable to create video encoder component");
        goto error;
    }

    if (!encoder->input_num || !encoder->output_num)
    {
        status = MMAL_ENOSYS;
        vcos_log_error("Video encoder doesn't have input/output ports");
        goto error;
    }

    encoder_input = encoder->input[0];
    encoder_output = encoder->output[0];

    // We want same format on input and output
    mmal_format_copy(encoder_output->format, encoder_input->format);

    // Only supporting H264 at the moment
    encoder_output->format->encoding = state->encoding;

    if(state->encoding == MMAL_ENCODING_H264)
    {
        if(state->level == MMAL_VIDEO_LEVEL_H264_4)
        {
            if(state->bitrate > MAX_BITRATE_LEVEL4)
            {
                fprintf(stderr, "Bitrate too high: Reducing to 25MBit/s\n");
                state->bitrate = MAX_BITRATE_LEVEL4;
            }
        }
        else
        {
            if(state->bitrate > MAX_BITRATE_LEVEL42)
            {
                fprintf(stderr, "Bitrate too high: Reducing to 62.5MBit/s\n");
                state->bitrate = MAX_BITRATE_LEVEL42;
            }
        }
    }
    else if(state->encoding == MMAL_ENCODING_MJPEG)
    {
        if(state->bitrate > MAX_BITRATE_MJPEG)
        {
            fprintf(stderr, "Bitrate too high: Reducing to 25MBit/s\n");
            state->bitrate = MAX_BITRATE_MJPEG;
        }
    }

    encoder_output->format->bitrate = state->bitrate;

    if (state->encoding == MMAL_ENCODING_H264)
        encoder_output->buffer_size = encoder_output->buffer_size_recommended;
    else
        encoder_output->buffer_size = 256<<10;


    if (encoder_output->buffer_size < encoder_output->buffer_size_min)
        encoder_output->buffer_size = encoder_output->buffer_size_min;

    encoder_output->buffer_num = encoder_output->buffer_num_recommended;

    if (encoder_output->buffer_num < encoder_output->buffer_num_min)
        encoder_output->buffer_num = encoder_output->buffer_num_min;

    // We need to set the frame rate on output to 0, to ensure it gets
    // updated correctly from the input framerate when port connected
    encoder_output->format->es->video.frame_rate.num = 0;
    encoder_output->format->es->video.frame_rate.den = 1;

    // Commit the port changes to the output port
    status = mmal_port_format_commit(encoder_output);

    if (status != MMAL_SUCCESS)
    {
        vcos_log_error("Unable to set format on video encoder output port");
        goto error;
    }

    // Set the rate control parameter
    if (0)
    {
        MMAL_PARAMETER_VIDEO_RATECONTROL_T param = {{ MMAL_PARAMETER_RATECONTROL, sizeof(param)}, MMAL_VIDEO_RATECONTROL_DEFAULT};
        status = mmal_port_parameter_set(encoder_output, &param.hdr);
        if (status != MMAL_SUCCESS)
        {
            vcos_log_error("Unable to set ratecontrol");
            goto error;
        }

    }

    if (state->encoding == MMAL_ENCODING_H264 &&
            state->intraperiod != -1)
    {
        MMAL_PARAMETER_UINT32_T param = {{ MMAL_PARAMETER_INTRAPERIOD, sizeof(param)}, (uint32_t)state->intraperiod};
        status = mmal_port_parameter_set(encoder_output, &param.hdr);
        if (status != MMAL_SUCCESS)
        {
            vcos_log_error("Unable to set intraperiod");
            goto error;
        }
    }

    if (state->encoding == MMAL_ENCODING_H264 &&
            state->quantisationParameter)
    {
        MMAL_PARAMETER_UINT32_T param = {{ MMAL_PARAMETER_VIDEO_ENCODE_INITIAL_QUANT, sizeof(param)}, (uint32_t)state->quantisationParameter};
        status = mmal_port_parameter_set(encoder_output, &param.hdr);
        if (status != MMAL_SUCCESS)
        {
            vcos_log_error("Unable to set initial QP");
            goto error;
        }

        MMAL_PARAMETER_UINT32_T param2 = {{ MMAL_PARAMETER_VIDEO_ENCODE_MIN_QUANT, sizeof(param)}, (uint32_t)state->quantisationParameter};
        status = mmal_port_parameter_set(encoder_output, &param2.hdr);
        if (status != MMAL_SUCCESS)
        {
            vcos_log_error("Unable to set min QP");
            goto error;
        }

        MMAL_PARAMETER_UINT32_T param3 = {{ MMAL_PARAMETER_VIDEO_ENCODE_MAX_QUANT, sizeof(param)}, (uint32_t)state->quantisationParameter};
        status = mmal_port_parameter_set(encoder_output, &param3.hdr);
        if (status != MMAL_SUCCESS)
        {
            vcos_log_error("Unable to set max QP");
            goto error;
        }

    }

    if (state->encoding == MMAL_ENCODING_H264)
    {
        MMAL_PARAMETER_VIDEO_PROFILE_T  param;
        param.hdr.id = MMAL_PARAMETER_PROFILE;
        param.hdr.size = sizeof(param);

        param.profile[0].profile = (MMAL_VIDEO_PROFILE_T)state->profile;

        if((VCOS_ALIGN_UP(state->width,16) >> 4) * (VCOS_ALIGN_UP(state->height,16) >> 4) * state->framerate > 245760)
        {
            if((VCOS_ALIGN_UP(state->width,16) >> 4) * (VCOS_ALIGN_UP(state->height,16) >> 4) * state->framerate <= 522240)
            {
                fprintf(stderr, "Too many macroblocks/s: Increasing H264 Level to 4.2\n");
                state->level=MMAL_VIDEO_LEVEL_H264_42;
            }
            else
            {
                vcos_log_error("Too many macroblocks/s requested");
                goto error;
            }
        }

        param.profile[0].level = (MMAL_VIDEO_LEVEL_T)state->level;

        status = mmal_port_parameter_set(encoder_output, &param.hdr);
        if (status != MMAL_SUCCESS)
        {
            vcos_log_error("Unable to set H264 profile");
            goto error;
        }
    }

    if (mmal_port_parameter_set_boolean(encoder_input, MMAL_PARAMETER_VIDEO_IMMUTABLE_INPUT, state->immutableInput) != MMAL_SUCCESS)
    {
        vcos_log_error("Unable to set immutable input flag");
        // Continue rather than abort..
    }

    //set INLINE HEADER flag to generate SPS and PPS for every IDR if requested
    if (mmal_port_parameter_set_boolean(encoder_output, MMAL_PARAMETER_VIDEO_ENCODE_INLINE_HEADER, INLINE_HEADERS) != MMAL_SUCCESS)
    {
        vcos_log_error("failed to set INLINE HEADER FLAG parameters");
        // Continue rather than abort..
    }

    // Adaptive intra refresh settings
    if (state->encoding == MMAL_ENCODING_H264 &&
            state->intra_refresh_type != -1)
    {
        MMAL_PARAMETER_VIDEO_INTRA_REFRESH_T  param;
        param.hdr.id = MMAL_PARAMETER_VIDEO_INTRA_REFRESH;
        param.hdr.size = sizeof(param);

        // Get first so we don't overwrite anything unexpectedly
        status = mmal_port_parameter_get(encoder_output, &param.hdr);
        if (status != MMAL_SUCCESS)
        {
            vcos_log_warn("Unable to get existing H264 intra-refresh values. Please update your firmware");
            // Set some defaults, don't just pass random stack data
            param.air_mbs = param.air_ref = param.cir_mbs = param.pir_mbs = 0;
        }

        param.refresh_mode = (MMAL_VIDEO_INTRA_REFRESH_T)state->intra_refresh_type;

        //if (state->intra_refresh_type == MMAL_VIDEO_INTRA_REFRESH_CYCLIC_MROWS)
        //   param.cir_mbs = 10;

        status = mmal_port_parameter_set(encoder_output, &param.hdr);
        if (status != MMAL_SUCCESS)
        {
            vcos_log_error("Unable to set H264 intra-refresh values");
            goto error;
        }
    }

    //  Enable component
    status = mmal_component_enable(encoder);

    if (status != MMAL_SUCCESS)
    {
        vcos_log_error("Unable to enable video encoder component");
        goto error;
    }

    /* Create pool of buffer headers for the output port to consume */
    pool = mmal_port_pool_create(encoder_output, encoder_output->buffer_num, encoder_output->buffer_size);

    if (!pool)
    {
        vcos_log_error("Failed to create buffer header pool for encoder output port %s", encoder_output->name);
    }

    state->encoder_pool = pool;
    state->encoder_component = encoder;

    if (state->verbose)
        fprintf(stderr, "Encoder component done\n");

    return status;

error:
    if (encoder)
        mmal_component_destroy(encoder);

    state->encoder_component = NULL;

    return status;
}

/**
 * Destroy the encoder component
 *
 * @param state Pointer to state control struct
 *
 */
static void destroy_encoder_component(RASPIVID_STATE *state)
{
    // Get rid of any port buffers first
    if (state->encoder_pool)
    {
        mmal_port_pool_destroy(state->encoder_component->output[0], state->encoder_pool);
    }

    if (state->encoder_component)
    {
        mmal_component_destroy(state->encoder_component);
        state->encoder_component = NULL;
    }
}

/**
 * Connect two specific ports together
 *
 * @param output_port Pointer the output port
 * @param input_port Pointer the input port
 * @param Pointer to a mmal connection pointer, reassigned if function successful
 * @return Returns a MMAL_STATUS_T giving result of operation
 *
 */
static MMAL_STATUS_T connect_ports(MMAL_PORT_T *output_port, MMAL_PORT_T *input_port, MMAL_CONNECTION_T **connection)
{
    MMAL_STATUS_T status;

    status =  mmal_connection_create(connection, output_port, input_port, MMAL_CONNECTION_FLAG_TUNNELLING | MMAL_CONNECTION_FLAG_ALLOCATION_ON_INPUT);

    if (status == MMAL_SUCCESS)
    {
        status =  mmal_connection_enable(*connection);
        if (status != MMAL_SUCCESS)
            mmal_connection_destroy(*connection);
    }

    return status;
}

/**
 * Checks if specified port is valid and enabled, then disables it
 *
 * @param port  Pointer the port
 *
 */
static void check_disable_port(MMAL_PORT_T *port)
{
    if (port && port->is_enabled)
        mmal_port_disable(port);
}

/**
 * Handler for sigint signals
 *
 * @param signal_number ID of incoming signal.
 *
 */
static void signal_handler(int signal_number)
{
    if (signal_number == SIGUSR1)
    {
        // Handle but ignore - prevents us dropping out if started in none-signal mode
        // and someone sends us the USR1 signal anyway
    }
    else
    {
        // Going to abort on all other signals
        vcos_log_error("Aborting program\n");
        exit(130);
    }

}

void set_recording(bool rec) {
    state.shouldRecord = rec ? 1 : 0;
    write_gpio(LIGHT_GPIO, rec ? 1 : 0);
}

void lights_out() {
    write_gpio(LIGHT_GPIO, 0);
}

volatile bool running = true;


/* reasonable for daylight */
float rgain = 1.7f;
float bgain = 1.5f;

void setup_logs() {
    char buf[100];
    time_t t;
    time(&t);
    if (!isatty(0)) {
        fprintf(stderr, "Running as script; logging to disk\n");
        strftime(buf, 100, "/var/tmp/mpq/stdout-%Y-%m-%d-%H-%M-%S.log", localtime(&t));
        freopen(buf, "w", stdout);
        strftime(buf, 100, "/var/tmp/mpq/stderr-%Y-%m-%d-%H-%M-%S.log", localtime(&t));
        freopen(buf, "w", stderr);
        freopen("/dev/null", "r", stdin);
    }
    strftime(buf, 100, "%Y-%m-%d %H:%M:%S", localtime(&t));
    fprintf(stderr, "%s: CamCam built %s %s\n", buf, __DATE__, __TIME__);
    fprintf(stderr, "user %d, group %d, pid %d\n", getuid(), getgid(), getpid());
}

/**
 * main
 */
int main(int argc, const char **argv)
{
    setenv("LOCALE", "C", 1);
    setup_logs();

    if (argv[1] && !strcmp(argv[1], "night")) {
        /* reasonable for incandescent */
        fprintf(stderr, "night color balance enabled\n");
        rgain = 1.3f;
        bgain = 2.6f;
        ++argv;
        --argc;
    }

    load_settings("camcam");

    int exit_code = EX_OK;

    MMAL_STATUS_T status = MMAL_SUCCESS;
    MMAL_PORT_T *camera_preview_port = NULL;
    MMAL_PORT_T *camera_video_port = NULL;
    MMAL_PORT_T *camera_still_port = NULL;
    MMAL_PORT_T *preview_input_port = NULL;
    MMAL_PORT_T *encoder_input_port = NULL;
    MMAL_PORT_T *encoder_output_port = NULL;
    MMAL_PORT_T *splitter_input_port = NULL;
    MMAL_PORT_T *splitter_output_port = NULL;
    MMAL_PORT_T *splitter_preview_port = NULL;

    bcm_host_init();

    // Register our application with the logging system
    vcos_log_register("RaspiVid", VCOS_LOG_CATEGORY);

    signal(SIGINT, signal_handler);

    // Disable USR1 for the moment - may be reenabled if go in to signal capture mode
    signal(SIGUSR1, SIG_IGN);

    default_status(&state);
    state.preview_parameters.wantFullScreenPreview = 0;
    state.preview_parameters.opacity = 255;
    state.preview_parameters.previewWindow.x = 0;
    state.preview_parameters.previewWindow.y = 800-PROC_HEIGHT;
    state.preview_parameters.previewWindow.width = PROC_WIDTH;
    state.preview_parameters.previewWindow.height = PROC_HEIGHT;
    state.camera_parameters.awbMode = MMAL_PARAM_AWBMODE_OFF;
    state.camera_parameters.awb_gains_r = rgain;
    state.camera_parameters.awb_gains_b = bgain;
    state.camera_parameters.flickerAvoidMode = MMAL_PARAM_FLICKERAVOID_60HZ;
    state.camera_parameters.exposureMode = MMAL_PARAM_EXPOSUREMODE_ANTISHAKE;
    state.camera_parameters.exposureMeterMode = MMAL_PARAM_EXPOSUREMETERINGMODE_MATRIX;
    state.camera_parameters.drc_level = MMAL_PARAMETER_DRC_STRENGTH_OFF;
    state.camera_parameters.videoStabilisation = 1;
    state.width = PROC_WIDTH;
    state.height = PROC_HEIGHT;
    state.raw_output_fmt = RAW_OUTPUT_FMT_YUV;
    state.bitrate = 0; //8000000;
    state.framerate = VIDEO_FRAME_RATE_NUM;
    state.intraperiod = 60;
    state.segmentSize = SEGMENT_SECONDS * 1000;
    state.quantisationParameter = QUANT_PARAM; //0; // 10 == extreme quality; 40 == good compression
    state.profile = MMAL_VIDEO_PROFILE_H264_HIGH;
    state.level = MMAL_VIDEO_LEVEL_H264_42;
    char buf[100];
    time_t t;
    time(&t);
    strftime(buf, 100, "/var/tmp/mpq/%Y-%m-%d-%H-%M-%S-video", localtime(&t));
    state.filename = strdup(buf);

    // Do we have any parameters
    if (argc > 1)
    {
        // Parse the command line and put options in to our status structure
        if (parse_cmdline(argc, argv, &state))
        {
            status = (MMAL_STATUS_T)-1;
            exit(EX_USAGE);
        }
    }

    if (state.verbose)
    {
        fprintf(stderr, "\n%s camcam %s\n\n", basename(argv[0]), VERSION_STRING);
        dump_status(&state);
    }

    create_main_window();

    // OK, we have a nice set of parameters. Now set up our components
    // We have three components. Camera, Preview and encoder.

    if ((status = create_camera_component(&state)) != MMAL_SUCCESS)
    {
        vcos_log_error("%s: Failed to create camera component", __func__);
        exit_code = EX_SOFTWARE;
    }
    else if ((status = raspipreview_create(&state.preview_parameters)) != MMAL_SUCCESS)
    {
        vcos_log_error("%s: Failed to create preview component", __func__);
        destroy_camera_component(&state);
        exit_code = EX_SOFTWARE;
    }
    else if ((status = create_encoder_component(&state)) != MMAL_SUCCESS)
    {
        vcos_log_error("%s: Failed to create encode component", __func__);
        raspipreview_destroy(&state.preview_parameters);
        destroy_camera_component(&state);
        exit_code = EX_SOFTWARE;
    }
    else if ((status = create_splitter_component(&state)) != MMAL_SUCCESS)
    {
        vcos_log_error("%s: Failed to create splitter component", __func__);
        raspipreview_destroy(&state.preview_parameters);
        destroy_camera_component(&state);
        destroy_encoder_component(&state);
        exit_code = EX_SOFTWARE;
    }
    else
    {
        if (state.verbose)
            fprintf(stderr, "Starting component connection stage\n");

        camera_preview_port = state.camera_component->output[MMAL_CAMERA_PREVIEW_PORT];
        camera_video_port   = state.camera_component->output[MMAL_CAMERA_VIDEO_PORT];
        camera_still_port   = state.camera_component->output[MMAL_CAMERA_CAPTURE_PORT];
        preview_input_port  = state.preview_parameters.preview_component->input[0];
        encoder_input_port  = state.encoder_component->input[0];
        encoder_output_port = state.encoder_component->output[0];

        splitter_input_port = state.splitter_component->input[0];
        splitter_output_port = state.splitter_component->output[SPLITTER_OUTPUT_PORT];
        splitter_preview_port = state.splitter_component->output[SPLITTER_PREVIEW_PORT];

        if (state.verbose)
            fprintf(stderr, "Connecting camera preview port to splitter input port\n");

        // Connect camera to splitter
        status = connect_ports(camera_preview_port, splitter_input_port, &state.splitter_connection);

        if (status != MMAL_SUCCESS)
        {
            state.splitter_connection = NULL;
            vcos_log_error("%s: Failed to connect camera preview port to splitter input", __func__);
            goto error;
        }

        if (state.verbose)
        {
            fprintf(stderr, "Connecting splitter preview port to preview input port\n");
            fprintf(stderr, "Starting video preview\n");
        }

        // Connect splitter to preview
        status = connect_ports(splitter_preview_port, preview_input_port, &state.preview_connection);

        if (status != MMAL_SUCCESS)
            state.preview_connection = NULL;

        if (status == MMAL_SUCCESS)
        {
            if (state.verbose)
                fprintf(stderr, "Connecting camera video port to encoder input port\n");

            // Now connect the camera to the encoder
            status = connect_ports(camera_video_port, encoder_input_port, &state.encoder_connection);

            if (status != MMAL_SUCCESS)
            {
                state.encoder_connection = NULL;
                vcos_log_error("%s: Failed to connect camera video port to encoder input", __func__);
                goto error;
            }
        }

        if (status == MMAL_SUCCESS)
        {
            // Set up our userdata - this is passed though to the callback where we need the information.
            state.callback_data.pstate = &state;
            state.callback_data.abort = 0;

            splitter_output_port->userdata = (struct MMAL_PORT_USERDATA_T *)&state.callback_data;

            if (state.verbose)
                fprintf(stderr, "Enabling splitter output port\n");

            // Enable the splitter output port and tell it its callback function
            status = mmal_port_enable(splitter_output_port, splitter_buffer_callback);

            if (status != MMAL_SUCCESS)
            {
                vcos_log_error("%s: Failed to setup splitter output port", __func__);
                goto error;
            }

            state.callback_data.file_handle = NULL;
            state.callback_data.raw_file_handle = NULL;

            if (state.raw_filename)
            {
                if (state.raw_filename[0] == '-')
                {
                    state.callback_data.raw_file_handle = stdout;
                }
                else
                {
                    state.callback_data.raw_file_handle = open_filename(&state, state.raw_filename);
                }
            }

            // Set up our userdata - this is passed though to the callback where we need the information.
            encoder_output_port->userdata = (struct MMAL_PORT_USERDATA_T *)&state.callback_data;

            if (state.verbose)
                fprintf(stderr, "Enabling encoder output port\n");

            // Enable the encoder output port and tell it its callback function
            status = mmal_port_enable(encoder_output_port, encoder_buffer_callback);

            if (status != MMAL_SUCCESS)
            {
                vcos_log_error("Failed to setup encoder output");
                goto error;
            }

            // Only encode stuff if we have a filename and it opened
            // Note we use the copy in the callback, as the call back MIGHT change the file handle
            if (!state.callback_data.file_handle)
            {
                fprintf(stderr, "Not capturing to file\n");
            }

            start_analyzer();

            // Send all the buffers to the encoder output port
            {
                int num = mmal_queue_length(state.encoder_pool->queue);
                int q;
                for (q=0;q<num;q++)
                {
                    MMAL_BUFFER_HEADER_T *buffer = mmal_queue_get(state.encoder_pool->queue);

                    if (!buffer)
                        vcos_log_error("Unable to get a required buffer %d from pool queue", q);

                    if (mmal_port_send_buffer(encoder_output_port, buffer)!= MMAL_SUCCESS)
                        vcos_log_error("Unable to send a buffer to encoder output port (%d)", q);
                }
            }

            // Send all the buffers to the splitter output port
            int num = mmal_queue_length(state.splitter_pool->queue);
            int q;
            for (q = 0; q < num; q++)
            {
                MMAL_BUFFER_HEADER_T *buffer = mmal_queue_get(state.splitter_pool->queue);

                if (!buffer)
                    vcos_log_error("Unable to get a required buffer %d from pool queue", q);

                if (mmal_port_send_buffer(splitter_output_port, buffer)!= MMAL_SUCCESS)
                    vcos_log_error("Unable to send a buffer to splitter output port (%d)", q);
            }

            if (mmal_port_parameter_set_boolean(camera_video_port, MMAL_PARAMETER_CAPTURE, 1) != MMAL_SUCCESS)
            {
                // How to handle?
                vcos_log_error("Could not turn on capturing");
            }

            atexit(lights_out);
            open_gpio(LIGHT_GPIO, true);

            run_sync_thread();
            run_main_loop();
            stop_sync_thread();

            if (state.verbose)
                fprintf(stderr, "Finished capture\n");

            save_settings("camcam");
            fprintf(stderr, "saved settings\n");
            stop_analyzer();
        }
        else
        {
            mmal_status_to_int(status);
            vcos_log_error("%s: Failed to connect camera to preview", __func__);
        }

error:

        mmal_status_to_int(status);

        if (state.verbose)
            fprintf(stderr, "Closing down\n");

        // Disable all our ports that are not handled by connections
        check_disable_port(camera_still_port);
        check_disable_port(encoder_output_port);
        check_disable_port(splitter_output_port);

        if (state.preview_parameters.wantPreview && state.preview_connection)
            mmal_connection_destroy(state.preview_connection);

        if (state.encoder_connection)
            mmal_connection_destroy(state.encoder_connection);

        if (state.splitter_connection)
            mmal_connection_destroy(state.splitter_connection);

        // Can now close our file. Note disabling ports may flush buffers which causes
        // problems if we have already closed the file!
        if (state.callback_data.file_handle && state.callback_data.file_handle != stdout)
            fclose(state.callback_data.file_handle);
        if (state.callback_data.raw_file_handle && state.callback_data.raw_file_handle != stdout)
            fclose(state.callback_data.raw_file_handle);

        /* Disable components */
        if (state.encoder_component)
            mmal_component_disable(state.encoder_component);

        if (state.preview_parameters.preview_component)
            mmal_component_disable(state.preview_parameters.preview_component);

        if (state.splitter_component)
            mmal_component_disable(state.splitter_component);

        if (state.camera_component)
            mmal_component_disable(state.camera_component);

        fprintf(stderr, "destroying components\n");

        destroy_encoder_component(&state);
        raspipreview_destroy(&state.preview_parameters);
        destroy_splitter_component(&state);
        destroy_camera_component(&state);

        if (state.verbose)
            fprintf(stderr, "Close down completed, all components disconnected, disabled and destroyed\n\n");
    }

    if (status != MMAL_SUCCESS)
        raspicamcontrol_check_configuration(128);

    fprintf(stderr, "return from main()");

    return exit_code;
}


void test_assert(bool b, char const *expr) {
    if (!b) {
        fprintf(stderr, "ASSERT FAILED: %s\n", expr);
        exit(1);
    }
}

/*
   Copyright (c) 2013, Broadcom Europe Ltd
   Copyright (c) 2013, James Hughes
   All rights reserved.

   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions are met:
 * Redistributions of source code must retain the above copyright
 notice, this list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright
 notice, this list of conditions and the following disclaimer in the
 documentation and/or other materials provided with the distribution.
 * Neither the name of the copyright holder nor the
 names of its contributors may be used to endorse or promote products
 derived from this software without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY
 DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * \file RaspiVid.c
 * Command line program to capture a camera video stream and encode it to file.
 * Also optionally display a preview/viewfinder of current camera input.
 *
 * \date 28th Feb 2013
 * \Author: James Hughes
 *
 * Description
 *
 * 3 components are created; camera, preview and video encoder.
 * Camera component has three ports, preview, video and stills.
 * This program connects preview and video to the preview and video
 * encoder. Using mmal we don't need to worry about buffers between these
 * components, but we do need to handle buffers from the encoder, which
 * are simply written straight to the file in the requisite buffer callback.
 *
 * If raw option is selected, a video splitter component is connected between
 * camera and preview. This allows us to set up callback for raw camera data
 * (in YUV420 or RGB format) which might be useful for further image processing.
 *
 * We use the RaspiCamControl code to handle the specific camera settings.
 * We use the RaspiPreview code to handle the (generic) preview window
 */


