#include <image_transport/simple_publisher_plugin.h>
#include <x264_image_transport/x264Packet.h>
#include <pthread.h>

#ifndef __APPLE__
//Dynamic reconfigure not yet working on iOS
#include <x264_image_transport/x264PublisherConfig.h>
#include <dynamic_reconfigure/server.h>
#endif

// ffmpeg is C library
extern "C" {
#include <libavcodec/avcodec.h>
#include <libavutil/avutil.h>
#include <libavutil/opt.h>
#include <libavutil/imgutils.h>
#include <libavutil/samplefmt.h>
#include <libswscale/swscale.h>
}


namespace x264_image_transport {

    class x264Publisher : public image_transport::SimplePublisherPlugin<x264_image_transport::x264Packet>
    {
    public:

        x264Publisher();

        ~x264Publisher() override;

        // Return the system unique string representing the x264 transport type
        std::string getTransportName() const override { return "x264"; }

    protected:

        // Overridden to tweak arguments and set up reconfigure server
        void advertiseImpl(ros::NodeHandle &nh, const std::string &base_topic, uint32_t queue_size,
                           const image_transport::SubscriberStatusCallback  &user_connect_cb,
                           const image_transport::SubscriberStatusCallback  &user_disconnect_cb,
                           const ros::VoidPtr &tracked_object, bool latch) override;

        // Callback to send header packets to new clients
        void connectCallback(const ros::SingleSubscriberPublisher& pub) override;

        // Main publish function
        void publish(const sensor_msgs::Image& message,
                             const PublishFn& publish_fn) const override;

        // Dynamic reconfigure support
#ifndef __APPLE__
        typedef x264_image_transport::x264PublisherConfig Config;
        typedef dynamic_reconfigure::Server<Config> ReconfigureServer;
        boost::shared_ptr<ReconfigureServer> reconfigure_server_;
        void configCb(Config& config, uint32_t level);
#endif

    private:
        void initialize_codec(int width,int height,int fps, const std::string &encoding) const;
        void memory_cleanup() const;

        /** ENCODER VAR **/
        mutable AVCodecContext  *enc_context_;
        mutable AVFrame *enc_frame_;
        mutable AVPacket enc_packet_;

        /** SOFTWARE SCALE CONTEXT **/
        mutable SwsContext *sws_context_;

        //initialized flag
        mutable bool initialized_;

        //Maximum quantization
        int quantization_max_;

        //Config protect
        mutable pthread_mutex_t mutex_;
    };

} //namespace x264_image_transport
