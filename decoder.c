#include "decoder.h"

#include <libavformat/avformat.h>

#include "events.h"
#include "video_buffer.h"
#include "trait/frame_sink.h"
#include "util/log.h"

/** Downcast packet_sink to decoder */
#define DOWNCAST(SINK) container_of(SINK, struct decoder, packet_sink)
#define RPI_H264_HDEC

#ifdef RPI_H264_HDEC
#include "rpi_h264.h"

    MMAL_STATUS_T status = MMAL_EINVAL;
    MMAL_COMPONENT_T *mmal_decoder = NULL;
    MMAL_POOL_T *pool_in = NULL, *pool_out = NULL;
    MMAL_BOOL_T eos_sent = MMAL_FALSE, eos_received = MMAL_FALSE;
    unsigned int in_count = 0, out_count = 0;
    int new_frame = 0;
    MMAL_BUFFER_HEADER_T *buffer;

/** Context for our application */
static struct CONTEXT_T {
   VCOS_SEMAPHORE_T semaphore;
   MMAL_QUEUE_T *queue;
   MMAL_STATUS_T status;
} context;

static void log_format(MMAL_ES_FORMAT_T *format, MMAL_PORT_T *port)
{
   const char *name_type;

   if(port)
      fprintf(stderr, "%s:%s:%i", port->component->name,
               port->type == MMAL_PORT_TYPE_CONTROL ? "ctr" :
                  port->type == MMAL_PORT_TYPE_INPUT ? "in" :
                  port->type == MMAL_PORT_TYPE_OUTPUT ? "out" : "invalid",
               (int)port->index);

   switch(format->type)
   {
   case MMAL_ES_TYPE_AUDIO: name_type = "audio"; break;
   case MMAL_ES_TYPE_VIDEO: name_type = "video"; break;
   case MMAL_ES_TYPE_SUBPICTURE: name_type = "subpicture"; break;
   default: name_type = "unknown"; break;
   }

   fprintf(stderr, "type: %s, fourcc: %4.4s\n", name_type, (char *)&format->encoding);
   fprintf(stderr, " bitrate: %i, framed: %i\n", format->bitrate,
            !!(format->flags & MMAL_ES_FORMAT_FLAG_FRAMED));
   fprintf(stderr, " extra data: %i, %p\n", format->extradata_size, format->extradata);
   switch(format->type)
   {
   case MMAL_ES_TYPE_AUDIO:
      fprintf(stderr, " samplerate: %i, channels: %i, bps: %i, block align: %i\n",
               format->es->audio.sample_rate, format->es->audio.channels,
               format->es->audio.bits_per_sample, format->es->audio.block_align);
      break;

   case MMAL_ES_TYPE_VIDEO:
      fprintf(stderr, " width: %i, height: %i, (%i,%i,%i,%i)\n",
               format->es->video.width, format->es->video.height,
               format->es->video.crop.x, format->es->video.crop.y,
               format->es->video.crop.width, format->es->video.crop.height);
      fprintf(stderr, " pixel aspect ratio: %i/%i, frame rate: %i/%i\n",
               format->es->video.par.num, format->es->video.par.den,
               format->es->video.frame_rate.num, format->es->video.frame_rate.den);
      break;

   case MMAL_ES_TYPE_SUBPICTURE:
      break;

   default: break;
   }

   if(!port)
      return;

   fprintf(stderr, " buffers num: %i(opt %i, min %i), size: %i(opt %i, min: %i), align: %i\n",
            port->buffer_num, port->buffer_num_recommended, port->buffer_num_min,
            port->buffer_size, port->buffer_size_recommended, port->buffer_size_min,
            port->buffer_alignment_min);
}

/** Callback from the control port.
 * Component is sending us an event. */
static void control_callback(MMAL_PORT_T *port, MMAL_BUFFER_HEADER_T *buffer)
{
   struct CONTEXT_T *ctx = (struct CONTEXT_T *)port->userdata;

   switch (buffer->cmd)
   {
   case MMAL_EVENT_EOS:
      /* Only sink component generate EOS events */
      break;
   case MMAL_EVENT_ERROR:
      /* Something went wrong. Signal this to the application */
      fprintf(stderr, "Error event signalled from %s, status %08X\n", port->name, *(MMAL_STATUS_T*)buffer->data);
      ctx->status = *(MMAL_STATUS_T *)buffer->data;
      break;
   default:
      break;
   }

   /* Done with the event, recycle it */
   mmal_buffer_header_release(buffer);

   /* Kick the processing thread */
   vcos_semaphore_post(&ctx->semaphore);
}

/** Callback from the input port.
 * Buffer has been consumed and is available to be used again. */
static void input_callback(MMAL_PORT_T *port, MMAL_BUFFER_HEADER_T *buffer)
{
   struct CONTEXT_T *ctx = (struct CONTEXT_T *)port->userdata;

   /* The mmal_decoder is done with the data, just recycle the buffer header into its pool */
   mmal_buffer_header_release(buffer);

   /* Kick the processing thread */
   vcos_semaphore_post(&ctx->semaphore);
}

/** Callback from the output port.
 * Buffer has been produced by the port and is available for processing. */
static void output_callback(MMAL_PORT_T *port, MMAL_BUFFER_HEADER_T *buffer)
{
   struct CONTEXT_T *ctx = (struct CONTEXT_T *)port->userdata;

   /* Queue the decoded video frame */
   mmal_queue_put(ctx->queue, buffer);

   /* Kick the processing thread */
   vcos_semaphore_post(&ctx->semaphore);

   /*fprintf(stderr, "decoded frame (flags %x, size %d, decoder %p) count %d - %d\n",
			  buffer->flags, buffer->length, globDecoder,in_count, out_count);*/
   new_frame++;
   out_count++;
}

#endif

static void
decoder_close_first_sinks(struct decoder *decoder, unsigned count) {
    while (count) {
        struct sc_frame_sink *sink = decoder->sinks[--count];
        sink->ops->close(sink);
    }
}

static inline void
decoder_close_sinks(struct decoder *decoder) {
    decoder_close_first_sinks(decoder, decoder->sink_count);
}

static bool
decoder_open_sinks(struct decoder *decoder) {
    for (unsigned i = 0; i < decoder->sink_count; ++i) {
        struct sc_frame_sink *sink = decoder->sinks[i];
        if (!sink->ops->open(sink)) {
            LOGE("Could not open frame sink %d", i);
            decoder_close_first_sinks(decoder, i);
            return false;
        }
    }

    return true;
}

static bool framebuff_alloc(struct decoder *decoder,int width, int height){
	int aligned_width = width + (PROC_ALIGNMENT - width%PROC_ALIGNMENT);
    int nBytes = avpicture_get_size(AV_PIX_FMT_YUV420P, aligned_width, height);
    const uint8_t *frBuffer = (const uint8_t *) av_malloc(nBytes);
    if(!frBuffer ){
    	av_frame_free(&decoder->frame);
        avcodec_close(decoder->codec_ctx);
        avcodec_free_context(&decoder->codec_ctx);
        return false;
    }
    avpicture_fill((AVPicture *)decoder->frame, frBuffer, AV_PIX_FMT_YUV420P,aligned_width, height);
    decoder->frame->linesize[0] = aligned_width;
    decoder->frame->linesize[1] = aligned_width/2;
    decoder->frame->linesize[2] = aligned_width/2;
    decoder->frame->width = width;
    decoder->frame->height = height;
    decoder->frame->format = AV_PIX_FMT_YUV420P;
    decoder->frame->pict_type = AV_PICTURE_TYPE_I;
    decoder->frame->pts = 0;
    //decoder->frame->pkt_pts = 0;
    decoder->frame->pkt_dts = 0;
    decoder->frame->color_range = AVCOL_RANGE_MPEG;
    decoder->frame->color_primaries = AVCOL_PRI_BT470BG;
    decoder->frame->color_trc = AVCOL_TRC_SMPTE170M;
    decoder->frame->colorspace = AVCHROMA_LOC_LEFT;
    return true;
}

static bool
decoder_open(struct decoder *decoder, const AVCodec *codec) {
    decoder->codec_ctx = avcodec_alloc_context3(codec);
    if (!decoder->codec_ctx) {
        LOGC("Could not allocate decoder context");
        return false;
    }

    if (avcodec_open2(decoder->codec_ctx, codec, NULL) < 0) {
        LOGE("Could not open codec");
        avcodec_free_context(&decoder->codec_ctx);
        return false;
    }

    decoder->frame = av_frame_alloc();
    if (!decoder->frame) {
        LOGE("Could not create decoder frame");
        avcodec_close(decoder->codec_ctx);
        avcodec_free_context(&decoder->codec_ctx);
        return false;
    }
    framebuff_alloc(decoder, 360, 800);

    if (!decoder_open_sinks(decoder)) {
        LOGE("Could not open decoder sinks");
        av_frame_free(&decoder->frame);
        avcodec_close(decoder->codec_ctx);
        avcodec_free_context(&decoder->codec_ctx);
        return false;
    }
    return true;
}

static void
decoder_close(struct decoder *decoder) {

    decoder_close_sinks(decoder);
    av_frame_free(&decoder->frame);
    avcodec_close(decoder->codec_ctx);
    avcodec_free_context(&decoder->codec_ctx);
#ifdef RPI_H264_HDEC
    /* Stop decoding */
    fprintf(stderr, "stop decoding\n");

    /* Stop everything. Not strictly necessary since mmal_component_destroy()
     * will do that anyway */
    mmal_port_disable(mmal_decoder->input[0]);
    mmal_port_disable(mmal_decoder->output[0]);
    mmal_component_disable(mmal_decoder);

  error:
    /* Cleanup everything */
    if (pool_in)
       mmal_port_pool_destroy(mmal_decoder->input[0], pool_in);
    if (pool_out)
       mmal_port_pool_destroy(mmal_decoder->output[0], pool_out);
    if (mmal_decoder)
       mmal_component_destroy(mmal_decoder);
    if (context.queue)
       mmal_queue_destroy(context.queue);


    vcos_semaphore_delete(&context.semaphore);
#endif
}

static bool
push_frame_to_sinks(struct decoder *decoder, const AVFrame *frame) {
    for (unsigned i = 0; i < decoder->sink_count; ++i) {
        struct sc_frame_sink *sink = decoder->sinks[i];
        if (!sink->ops->push(sink, frame)) {
            LOGE("Could not send frame to sink %d", i);
            return false;
        }
    }

    return true;
}

//HERE ENDS ALL AV PACKETS
static bool
decoder_push(struct decoder *decoder, const AVPacket *packet) {
#ifdef RPI_H264_HDEC
    VCOS_STATUS_T vcos_status;

    /* Wait for buffer headers to be available on either of the mmal_decoder ports */
    vcos_status = vcos_semaphore_wait_timeout(&context.semaphore, 2000);
    if (vcos_status != VCOS_SUCCESS)
       fprintf(stderr, "vcos_semaphore_wait_timeout failed - status %d\n", vcos_status);

    /* Check for errors */
    if (context.status != MMAL_SUCCESS)
       return false;
    static int frame_diffcount = 0;
    /* Send data to decode to the input port of the video mmal_decoder */
    if ((packet != NULL ) && (buffer = mmal_queue_get(pool_in->queue)) != NULL)
    {
    	//FROM FFMEGS' PACKET TO MMAL'S BUFFER
    	//PUSH DATA TO HARDWARE DECODER

    	//WE get errors.... Error event signalled... status 7
    	//Possible reasons:
    	//1. too large packet > 80000 causes Error

    	if(packet->size > 80000){
    		fprintf(stderr,"IN:%d - OUT:%d. packet size: %d\n", in_count, out_count,packet->size);
    	}
		buffer->data = packet->data;
	    buffer->length = packet->size;


	    if(!buffer->length) eos_sent = MMAL_TRUE;

	    buffer->flags = buffer->length ? 0 : MMAL_BUFFER_HEADER_FLAG_EOS;
	    buffer->pts = buffer->dts = MMAL_TIME_UNKNOWN;
		   //fprintf(stderr, "sending %i bytes\n", (int)buffer->length);
	    status = mmal_port_send_buffer(mmal_decoder->input[0], buffer);
	    CHECK_STATUS(status, "failed to send buffer");
	    in_count++;

    }

    /* Get our output frames */
    while ((buffer = mmal_queue_get(context.queue)) != NULL)
    {
       /* We have a frame, do something with it (why not display it for instance?).
        * Once we're done with it, we release it. It will automatically go back
        * to its original pool so it can be reused for a new video frame.
        */
       eos_received = buffer->flags & MMAL_BUFFER_HEADER_FLAG_EOS;

       if (buffer->cmd)
       {
          fprintf(stderr, "received event length %d, %4.4s\n", buffer->length, (char *)&buffer->cmd);
          if (buffer->cmd == MMAL_EVENT_FORMAT_CHANGED)
          {
             MMAL_EVENT_FORMAT_CHANGED_T *event = mmal_event_format_changed_get(buffer);
             if (event)
             {
                fprintf(stderr, "----------Port format changed----------\n");
                log_format(mmal_decoder->output[0]->format, mmal_decoder->output[0]);
                fprintf(stderr, "-----------------to---------------------\n");
                log_format(event->format, 0);
                fprintf(stderr, " buffers num (opt %i, min %i), size (opt %i, min: %i)\n",
                         event->buffer_num_recommended, event->buffer_num_min,
                         event->buffer_size_recommended, event->buffer_size_min);
                fprintf(stderr, "----------------------------------------\n");
             }
             mmal_buffer_header_release(buffer);
             mmal_port_disable(mmal_decoder->output[0]);

             //Clear out the queue and release the buffers.
             while(mmal_queue_length(pool_out->queue) < pool_out->headers_num)
             {
                buffer = mmal_queue_wait(context.queue);
                mmal_buffer_header_release(buffer);
                fprintf(stderr, "Retrieved buffer %p\n", buffer);
             }

             //Assume we can't reuse the output buffers, so have to disable, destroy
             //pool, create new pool, enable port, feed in buffers.
             mmal_port_pool_destroy(mmal_decoder->output[0], pool_out);

             status = mmal_format_full_copy(mmal_decoder->output[0]->format, event->format);
             mmal_decoder->output[0]->format->encoding = MMAL_ENCODING_I420;
             mmal_decoder->output[0]->buffer_num = MAX_BUFFERS;
             mmal_decoder->output[0]->buffer_size = mmal_decoder->output[0]->buffer_size_recommended;

             if (status == MMAL_SUCCESS)
                status = mmal_port_format_commit(mmal_decoder->output[0]);
             if (status != MMAL_SUCCESS)
             {
                fprintf(stderr, "commit failed on output - %d\n", status);
             }

             mmal_port_enable(mmal_decoder->output[0], output_callback);
             pool_out = mmal_port_pool_create(mmal_decoder->output[0], mmal_decoder->output[0]->buffer_num, mmal_decoder->output[0]->buffer_size);
          }
          else
          {
             mmal_buffer_header_release(buffer);
          }
          continue;
       }
       else
       {
          // Do something here with the content of the buffer
          //from the mmal we get 460800 bytes YUV420
          //data[0] - 307200 = 384*800*6/8 8bita po pixelu
          //data[1] - 76800 = 192*800*6/8 2bita po pixelu
          //data[2] - 76800 = 192*800*6/8 2bita po pixelu
          AVFrame *fr = decoder->frame;
/*COPY DATA*/
          /*
          memcpy(fr->data[1],buffer->data + buffer->length*2/3,buffer->length/6);
          memcpy(fr->data[2],buffer->data + buffer->length*5/6,buffer->length/6);
          memcpy(fr->data[0],buffer->data, buffer->length*2/3);
*/
          //TEST - FILL WITH ZEROS
          //memset(fr->data[0] + buffer->length/2, 0, buffer->length/6);
          //END TEST

          decoder->frame->data[0] = buffer->data;
          decoder->frame->data[1] = buffer->data + buffer->length*2/3;
          decoder->frame->data[2] = buffer->data + buffer->length*5/6;

 /*         LOGE("AVFrame: %p %d %p %d %p %d\n%p\n%d x %d\nFormat: %d\nKeyFrame: %d\n Crop: (%d, %d, %d, %d)\n%d",
          		fr->data[0], //0xa6ab4020,...
          		fr->linesize[0], //384
  				fr->data[1], //0xa6b681c0,....
          		fr->linesize[1],
  				fr->data[2],
          		fr->linesize[2],
  				fr->extended_data, //0xa6b24d90
  				fr->width, fr->height, //360x 800
  				fr->format,
  				fr->key_frame,
  				fr->crop_left, fr->crop_top, fr->crop_right, fr->crop_bottom,
  				fr->sample_rate
  				);*/
          bool ok = push_frame_to_sinks(decoder, decoder->frame);
          //following function resets all data in AVFrame and only first frame is valid
          //we use one AVFrame object for all frames (no unref commited).
          //av_frame_unref(decoder->frame);
          mmal_buffer_header_release(buffer);

          new_frame--;
       }
    }
    if((in_count - out_count) > frame_diffcount){
  	  frame_diffcount = (in_count - out_count);
  	  fprintf(stderr,"Diff increasing: %d = %u - %u\n", frame_diffcount, in_count, out_count);
    }
    /* Send empty buffers to the output port of the mmal_decoder */
    while ((buffer = mmal_queue_get(pool_out->queue)) != NULL)
    {
       //printf("Sending buf %p\n", buffer);
       status = mmal_port_send_buffer(mmal_decoder->output[0], buffer);
       CHECK_STATUS(status, "failed to send output buffer to mmal_decoder");
    }

#else
    bool is_config = packet->pts == AV_NOPTS_VALUE;
    if (is_config) {
        // nothing to do
        return true;
    }
    static int count = 0;
    if(count %20 == 25){
    LOGD("AVPacket: %d, %d, data: %d, Size: %d, \n%d \n%d\n%d %d",
    		packet->pts,  //52964249
			packet->dts,  //0
			packet->data[0], //52964249
			packet->size,//0
			packet->stream_index,//0
			packet->flags, //432,416,464,480,544...
			packet->duration, //0
			packet->pos); //0
    }
    ////SEND ENCODED DATA TO ENCODER
    int ret = avcodec_send_packet(decoder->codec_ctx, packet);

    if (ret < 0 && ret != AVERROR(EAGAIN)) {
        LOGE("Could not send video packet: %d", ret);
        return false;
    }

    //RECEIVE FRAME
    ret = avcodec_receive_frame(decoder->codec_ctx, decoder->frame);
    if (!ret) {
        // a frame was received
        bool ok = push_frame_to_sinks(decoder, decoder->frame);
        // A frame lost should not make the whole pipeline fail. The error, if
        // any, is already logged.
        (void) ok;
        AVFrame *fr = decoder->frame;
        if(count % 1 == 0){
        LOGE("AVFrame: %p %d %p %d %p %d\n%p\n%d x %d\nFormat: %d\nKeyFrame: %d\n Crop: (%d, %d, %d, %d)\n%d",
        		fr->data[0], //0xa6ab4020,...
        		fr->linesize[0], //384
				fr->data[1], //0xa6b681c0,....
        		fr->linesize[1],
				fr->data[2],
        		fr->linesize[2],
				fr->extended_data, //0xa6b24d90
				fr->width, fr->height, //360x 800
				fr->format,
				fr->key_frame,
				fr->crop_left, fr->crop_top, fr->crop_right, fr->crop_bottom,
				fr->sample_rate
				);
        }

        av_frame_unref(decoder->frame);
    } else if (ret != AVERROR(EAGAIN)) {
        LOGE("Could not receive video frame: %d", ret);
        return false;
    }
    count++;
#endif
    return true;
error:
	return false;
}

static bool
decoder_packet_sink_open(struct sc_packet_sink *sink, const AVCodec *codec) {
    struct decoder *decoder = DOWNCAST(sink);
    return decoder_open(decoder, codec);
}

static void
decoder_packet_sink_close(struct sc_packet_sink *sink) {
    struct decoder *decoder = DOWNCAST(sink);
    decoder_close(decoder);
}

static bool
decoder_packet_sink_push(struct sc_packet_sink *sink, const AVPacket *packet) {
    struct decoder *decoder = DOWNCAST(sink);
    return decoder_push(decoder, packet);
}

void
decoder_init(struct decoder *decoder) {
    decoder->sink_count = 0;

    static const struct sc_packet_sink_ops ops = {
        .open = decoder_packet_sink_open,
        .close = decoder_packet_sink_close,
        .push = decoder_packet_sink_push,
    };

    decoder->packet_sink.ops = &ops;


#ifdef 	RPI_H264_HDEC

    bcm_host_init();

 //   vcsm_init();

    vcos_semaphore_create(&context.semaphore, "example", 1);

    /* Create the mmal_decoder component.
     * This specific component exposes 2 ports (1 input and 1 output). Like most components
     * its expects the format of its input port to be set by the client in order for it to
     * know what kind of data it will be fed. */
    status = mmal_component_create(MMAL_COMPONENT_DEFAULT_VIDEO_DECODER, &mmal_decoder);
    CHECK_STATUS(status, "failed to create mmal_decoder");

    /* Enable control port so we can receive events from the component */
    mmal_decoder->control->userdata = (void *)&context;
    status = mmal_port_enable(mmal_decoder->control, control_callback);
    CHECK_STATUS(status, "failed to enable control port");

    /* Set the zero-copy parameter on the input port */
 //   status = mmal_port_parameter_set_boolean(mmal_decoder->input[0], MMAL_PARAMETER_ZERO_COPY, MMAL_TRUE);
 //   CHECK_STATUS(status, "failed to set zero copy - %s", mmal_decoder->input[0]->name);

    /* Set the zero-copy parameter on the output port */
    status = mmal_port_parameter_set_boolean(mmal_decoder->output[0], MMAL_PARAMETER_ZERO_COPY, MMAL_TRUE);
    CHECK_STATUS(status, "failed to set zero copy - %s", mmal_decoder->output[0]->name);

    /* Set format of video mmal_decoder input port */
    MMAL_ES_FORMAT_T *format_in = mmal_decoder->input[0]->format;
    format_in->type = MMAL_ES_TYPE_VIDEO;
    format_in->encoding = MMAL_ENCODING_H264;
    format_in->es->video.width = 360;
    format_in->es->video.height = 800;

    status = mmal_port_format_commit(mmal_decoder->input[0]);
    CHECK_STATUS(status, "failed to commit format");

    MMAL_ES_FORMAT_T *format_out = mmal_decoder->output[0]->format;
    format_out->encoding = MMAL_ENCODING_I420;

    status = mmal_port_format_commit(mmal_decoder->output[0]);
    CHECK_STATUS(status, "failed to commit format");

    /* Display the output port format */
    fprintf(stderr, "%s\n", mmal_decoder->output[0]->name);
    fprintf(stderr, " type: %i, fourcc: %4.4s\n", format_out->type, (char *)&format_out->encoding);
    fprintf(stderr, " bitrate: %i, framed: %i\n", format_out->bitrate,
            !!(format_out->flags & MMAL_ES_FORMAT_FLAG_FRAMED));
    fprintf(stderr, " extra data: %i, %p\n", format_out->extradata_size, format_out->extradata);
    fprintf(stderr, " width: %i, height: %i, (%i,%i,%i,%i)\n",
            format_out->es->video.width, format_out->es->video.height,
            format_out->es->video.crop.x, format_out->es->video.crop.y,
            format_out->es->video.crop.width, format_out->es->video.crop.height);

    /* The format of both ports is now set so we can get their buffer requirements and create
     * our buffer headers. We use the buffer pool API to create these. */
    mmal_decoder->input[0]->buffer_num = mmal_decoder->input[0]->buffer_num_recommended;
    mmal_decoder->input[0]->buffer_size = 100000;
    mmal_decoder->output[0]->buffer_num = mmal_decoder->output[0]->buffer_num_recommended;
    mmal_decoder->output[0]->buffer_size = mmal_decoder->output[0]->buffer_size_recommended;
    pool_in = mmal_port_pool_create(mmal_decoder->input[0],
                                    mmal_decoder->input[0]->buffer_num,
                                    mmal_decoder->input[0]->buffer_size);

    /* Create a queue to store our decoded frame(s). The callback we will get when
     * a frame has been decoded will put the frame into this queue. */
    context.queue = mmal_queue_create();

    /* Store a reference to our context in each port (will be used during callbacks) */
    mmal_decoder->input[0]->userdata = (void *)&context;
    mmal_decoder->output[0]->userdata = (void *)&context;

    /* Enable all the input port and the output port.
     * The callback specified here is the function which will be called when the buffer header
     * we sent to the component has been processed. */
    status = mmal_port_enable(mmal_decoder->input[0], input_callback);
    CHECK_STATUS(status, "failed to enable input port");

    status = mmal_port_enable(mmal_decoder->output[0], output_callback);
    CHECK_STATUS(status, "failed to enable output port");

    pool_out = mmal_port_pool_create(mmal_decoder->output[0],
                                 mmal_decoder->output[0]->buffer_num,
                                 mmal_decoder->output[0]->buffer_size);

    while ((buffer = mmal_queue_get(pool_out->queue)) != NULL)
    {
       //printf("Sending buf %p\n", buffer);
       status = mmal_port_send_buffer(mmal_decoder->output[0], buffer);
       CHECK_STATUS(status, "failed to send output buffer to mmal_decoder");
    }

    /* Component won't start processing data until it is enabled. */
    status = mmal_component_enable(mmal_decoder);
    CHECK_STATUS(status, "failed to enable mmal_decoder component");
    log_format(mmal_decoder->output[0]->format, mmal_decoder->output[0]);

    /* Start decoding */
    fprintf(stderr, "start decoding\n");
error:
	return;
#endif
}

void
decoder_add_sink(struct decoder *decoder, struct sc_frame_sink *sink) {
    assert(decoder->sink_count < DECODER_MAX_SINKS);
    assert(sink);
    assert(sink->ops);
    decoder->sinks[decoder->sink_count++] = sink;
}
