#if !defined(detect_h)
#define detect_h

#if defined(__cplusplus)
class FrameQueue;
extern FrameQueue analyzer_analyzed_queue;
#if !defined(DETECT_EXTERN)
#define DETECT_EXTERN extern "C"
#endif
#else
#if !defined(DETECT_EXTERN)
#define DETECT_EXTERN
#endif
#endif

struct MMAL_PORT_T;
struct MMAL_BUFFER_HEADER_T;
struct DetectOutput;

DETECT_EXTERN int analyzer_consume(struct MMAL_PORT_T *port, struct MMAL_BUFFER_HEADER_T *buffer);
DETECT_EXTERN void recycle_buffer(struct MMAL_PORT_T *port, struct MMAL_BUFFER_HEADER_T *buffer);
DETECT_EXTERN void start_analyzer();
DETECT_EXTERN void stop_analyzer();
DETECT_EXTERN void detect_write_params(char const *filename);
DETECT_EXTERN void detect_get_last_output(struct DetectOutput *output);
extern void *volatile browse_buffer;

#endif

