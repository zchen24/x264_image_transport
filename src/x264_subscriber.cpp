#include "x264_image_transport/x264_subscriber.h"
#include <sensor_msgs/image_encodings.h>
#include <boost/scoped_array.hpp>
#include <vector>


//using namespace std;
namespace x264_image_transport {

    namespace enc = sensor_msgs::image_encodings;

    x264Subscriber::x264Subscriber():
            codec_ctx_(nullptr),
            codec_(nullptr),
            av_frame_(nullptr),
            av_frame_RGB_(nullptr),
            sws_context_(nullptr),
            latest_image_(new sensor_msgs::Image()), initialized_(false)
    {
    }

    void x264Subscriber::initialize_codec(int width, int height)
    {
        ROS_INFO("x264Subscriber::initialize_codec(int width = %i, int height = %i)",width,height);


        // must be called before using avcodec lib
        avcodec_register_all();
        //Register codecs, devices and formats
        av_register_all();
        //Initialize network, this is new from april 2013, it will initialize the RTP
        avformat_network_init();

        //HARDCODING IMAGE SIZE
        latest_image_->height = static_cast<sensor_msgs::Image::_height_type>(height);
        latest_image_->width = static_cast<sensor_msgs::Image::_width_type >(width);
        latest_image_->step = latest_image_->width*3;
        latest_image_->data.resize(latest_image_->step * latest_image_->height);
        latest_image_->encoding = enc::RGB8;

        //Setting up codec information
        //------------------------------------------------------------------------------
        //Set the codec to H264
        ROS_INFO("VideoOutputContext::initialize : Setting AVCodec");
        codec_=avcodec_find_decoder(CODEC_ID_H264);

        if(codec_==nullptr) {
            ROS_ERROR("VideoOutputContext::initialize : Unsupported codec");
            return;
        }

        //Set the codec Context
        ROS_INFO("VideoOutputContext::initialize : Setting AVCodecContext");
        codec_ctx_ = avcodec_alloc_context3(codec_);
        codec_ctx_->width = latest_image_->width;
        codec_ctx_->height= latest_image_->height;
        codec_ctx_->pix_fmt = PIX_FMT_YUV420P;
        codec_ctx_->codec_id = CODEC_ID_H264;
        codec_ctx_->codec_type = AVMEDIA_TYPE_VIDEO;

/*
        codec_ctx_->error_concealment = 3;
        codec_ctx_->error_recognition =1;
        codec_ctx_->skip_loop_filter = AVDISCARD_DEFAULT;
        codec_ctx_->workaround_bugs = 1;
        codec_ctx_->sample_fmt = SAMPLE_FMT_NONE;
        codec_ctx_->skip_idct = AVDISCARD_DEFAULT;
        codec_ctx_->skip_frame = AVDISCARD_DEFAULT;
        codec_ctx_->color_primaries = AVCOL_PRI_UNSPECIFIED;
        //codec_ctx_->color_trc = AVCOL_PRI_UNSPECIFIED;
        //codec_ctx_->colorspace = AVCOL_PRI_UNSPECIFIED;
        codec_ctx_->color_range = AVCOL_RANGE_UNSPECIFIED;
        codec_ctx_->chroma_sample_location = AVCHROMA_LOC_LEFT;
        //codec_ctx_->lpc_type = AV_LPC_TYPE_DEFAULT;
        codec_ctx_->profile = 100;
        codec_ctx_->level = 40;
*/

        // Open codec
        ROS_INFO("Opening Codec");

        /* open the decoder codec */
        if (avcodec_open2(codec_ctx_, codec_, nullptr) < 0)
        {
            ROS_ERROR("Could not open the decoder");
            return;
        }

        // Allocate an AVFrame structure
        av_frame_ = av_frame_alloc();
        av_frame_RGB_ = av_frame_alloc();

        if(av_frame_ == nullptr || av_frame_RGB_== nullptr)
        {
            ROS_ERROR("Cannot allocate frame");
            return;
        }
        avpicture_alloc((AVPicture *)av_frame_RGB_, PIX_FMT_RGB24, codec_ctx_->width, codec_ctx_->height);

        initialized_ = true;
        ROS_INFO("x264Subscriber::initialize_codec() Codec Ready! v001");
    }

    x264Subscriber::~x264Subscriber()
    {
        initialized_ = false;

        if (codec_ctx_)
        {
            avcodec_close(codec_ctx_);
        }

        if(av_frame_)
        {
            av_free(av_frame_);
        }

        if (av_frame_RGB_)
        {
            av_free(av_frame_RGB_);
        }

        if (sws_context_)
        {
            sws_freeContext(sws_context_);
        }

    }

    void x264Subscriber::subscribeImpl(ros::NodeHandle &nh, const std::string &base_topic, uint32_t queue_size,
                                       const Callback &callback, const ros::VoidPtr &tracked_object,
                                       const image_transport::TransportHints &transport_hints)
    {
        //TODO UNDERSTAND THOSE PARAMETERS...


        // queue_size doesn't account for the 3 header packets, so we correct (with a little extra) here.
        queue_size += 4;
        typedef image_transport::SimpleSubscriberPlugin<x264_image_transport::x264Packet> Base;
        Base::subscribeImpl(nh, base_topic, queue_size, callback, tracked_object, transport_hints);

        // Set up reconfigure server for this topic
        reconfigure_server_ = boost::make_shared<ReconfigureServer>(this->nh());
        ReconfigureServer::CallbackType f = boost::bind(&x264Subscriber::configCb, this, _1, _2);
        reconfigure_server_->setCallback(f);
    }

    void x264Subscriber::configCb(Config& config, uint32_t level)
    {
        //TODO HANDLE CONFIGURATION
        ROS_INFO("x264Subscriber::configCb not implemented yet");
    }

    void x264Subscriber::convert_rgb(AVCodecContext *codec, AVFrame *inFrame, AVFrame *outFrame)
    {
        //Initialize converter context if required
        if (!sws_context_)
        {
            sws_context_ =sws_getContext(codec->width, codec->height, codec->pix_fmt, //src
                                         codec->width, codec->height, PIX_FMT_RGB24, //dest
                                         SWS_FAST_BILINEAR, nullptr, nullptr, nullptr);
        }

        //convert to RGB
        sws_scale(sws_context_, inFrame->data,
                  inFrame->linesize, 0,
                  codec->height,
                  outFrame->data, outFrame->linesize);
    }


    void x264Subscriber::internalCallback(const x264_image_transport::x264PacketConstPtr& msg, const Callback& user_cb)
    {
        //ROS_INFO("x264Subscriber::internalCallback");

        if (!initialized_)
        {
            initialize_codec(msg->img_width, msg->img_height);
        }

        //Something went wrong
        if (!initialized_) {
            return;
        }

        //decode video...
        //Create packet
        AVPacket packet;
        av_init_packet(&packet);

        //allocate packet memory
        av_new_packet(&packet, static_cast<int>(msg->data.size()));

        //copy data
        memcpy(packet.data, &msg->data[0], msg->data.size());


        //try decoding...
        int got_frame = 0;
        int result = avcodec_decode_video2(codec_ctx_, av_frame_, &got_frame, &packet);

        if(result >= 0 && got_frame > 0)
        {
            //ROS_INFO("Decoding result : %i got_frame : %i",result,got_frame);

            //Convert input image to RGB32 format
            convert_rgb(codec_ctx_,av_frame_,av_frame_RGB_);

            //Copy data (RGB24)
            memcpy(&latest_image_->data[0], av_frame_RGB_->data[0],
                   static_cast<size_t >(codec_ctx_->width *codec_ctx_->height * 3));

            //Affect header
            latest_image_->header = msg->header;

            //Call callback (new image received)
            user_cb(latest_image_);
        }

        //free packet
        av_free_packet(&packet);
    }


} //namespace x264_image_transport
