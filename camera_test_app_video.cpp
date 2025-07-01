#include <iomanip>
#include <unistd.h>
#include <iostream>
#include <memory>
#include <thread>
#include <mutex>
#include <chrono>
#include <condition_variable>
#include <sys/mman.h>
#include <syslog.h>

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libswscale/swscale.h>
#include <libavutil/imgutils.h>
}

#include <libcamera/libcamera.h>
#include <libcamera/controls.h>
#include <linux/videodev2.h> 
#include <opencv2/opencv.hpp>
#include <libcamera/control_ids.h>


using namespace libcamera;
using namespace std::chrono_literals;

int height; 
int width;
bool hflip;
bool vflip;

int durationSeconds;

double analog_gain;
int exposure;
int mode;


struct mode_struct {
        int bitDepth;
        int width;
        int height;
        int binning;
        int cropLeft;
        int cropTop;
        int cropWidth;
        int cropHeight;
        int fps;
};

static const mode_struct modes[] = {
        {.bitDepth= 12, .width= 4056, .height= 3040, .binning = 1, .cropLeft =   8, .cropTop =  16, .cropWidth = 4056, .cropHeight = 3040, .fps = 10},
        {.bitDepth= 12, .width= 2028, .height= 1520, .binning = 2, .cropLeft =   8, .cropTop =  16, .cropWidth = 4056, .cropHeight = 3040, .fps = (30 / 10)},
        {.bitDepth= 12, .width= 2028, .height= 1080, .binning = 2, .cropLeft =   8, .cropTop = 456, .cropWidth = 4056, .cropHeight = 2160, .fps = (40 / 10)},
        {.bitDepth= 10, .width= 1332, .height= 990 , .binning = 2, .cropLeft = 704, .cropTop = 544, .cropWidth = 2664, .cropHeight = 1980, .fps = (120 / 10)}
    
};

AVFormatContext *formatContext = nullptr;
AVCodecContext *codecContext = nullptr;
AVStream *videoStream = nullptr;
SwsContext *swsContext = nullptr;
AVFrame *yuvFrame = nullptr;
int64_t pts = 0;

void initFFmpeg(const char *filename) {
    if (avformat_alloc_output_context2(&formatContext, nullptr, nullptr, filename) < 0)
        throw std::runtime_error("Could not allocate format context");

    const AVCodec *codec = avcodec_find_encoder(AV_CODEC_ID_H264);
    if (!codec)
        throw std::runtime_error("H.264 encoder not found");

    codecContext = avcodec_alloc_context3(codec);
    codecContext->width = width;
    codecContext->height = height;
    codecContext->time_base = {1, modes[mode].fps};
    codecContext->framerate = {modes[mode].fps, 1};
    codecContext->pix_fmt = AV_PIX_FMT_YUV420P;
    codecContext->bit_rate = 400'000;

    if (formatContext->oformat->flags & AVFMT_GLOBALHEADER)
        codecContext->flags |= AV_CODEC_FLAG_GLOBAL_HEADER;
    
    AVDictionary* param = nullptr;
    av_dict_set(&param, "present", "ultrafast", 0);

    if (avcodec_open2(codecContext, codec, &param) < 0)
        throw std::runtime_error("Failed to open codec");

    videoStream = avformat_new_stream(formatContext, nullptr);
    if (!videoStream)
        throw std::runtime_error("Failed to create stream");

    videoStream->time_base = codecContext->time_base;
    if (avcodec_parameters_from_context(videoStream->codecpar, codecContext) < 0)
        throw std::runtime_error("Failed to copy codec parameters");

    if (avio_open(&formatContext->pb, filename, AVIO_FLAG_WRITE) < 0)
        throw std::runtime_error("Failed to open output file");

    if (avformat_write_header(formatContext, nullptr) < 0)
        throw std::runtime_error("Failed to write header");

    swsContext = sws_getContext(codecContext->width, codecContext->height, AV_PIX_FMT_RGB32,
                                codecContext->width, codecContext->height, AV_PIX_FMT_YUV420P,
                                SWS_BILINEAR, nullptr, nullptr, nullptr);

    yuvFrame = av_frame_alloc();
    yuvFrame->format = AV_PIX_FMT_YUV420P;
    yuvFrame->width = codecContext->width;
    yuvFrame->height = codecContext->height;
    if (av_frame_get_buffer(yuvFrame, 32) < 0)
        throw std::runtime_error("Failed to allocate frame buffer");
}

void cleanupFFmpeg() {
    // Flush encoder
    avcodec_send_frame(codecContext, nullptr);

    AVPacket *pkt = av_packet_alloc();
    if (!pkt) {
        std::cerr << "Failed to allocate AVPacket\n";
        return;
    }

    while (avcodec_receive_packet(codecContext, pkt) == 0) {
        av_interleaved_write_frame(formatContext, pkt);
        av_packet_unref(pkt);
    }

    av_packet_free(&pkt);  // Free the allocated packet

    av_write_trailer(formatContext);
    avcodec_free_context(&codecContext);
    avio_closep(&formatContext->pb);
    avformat_free_context(formatContext);
    av_frame_free(&yuvFrame);
    sws_freeContext(swsContext);
}

void encodeFrame(uint8_t *paddedData)
{
    size_t rowSize = width * 4;
    int numberUnusedBytes = (0x40 - (rowSize % 0x40)) % 0x40;
    size_t paddedRowSize = rowSize + numberUnusedBytes;

    uint8_t *src[1] = { paddedData };
    int srcStride[1] = { static_cast<int>(paddedRowSize) };

    sws_scale(swsContext, src, srcStride, 0, height,
              yuvFrame->data, yuvFrame->linesize);

    yuvFrame->pts = pts++;

    AVPacket *pkt = av_packet_alloc();
    if (!pkt) {
        std::cerr << "Failed to allocate AVPacket\n";
        return;
    }

    if (avcodec_send_frame(codecContext, yuvFrame) == 0) {
        while (avcodec_receive_packet(codecContext, pkt) == 0) {
            pkt->stream_index = videoStream->index;
            av_packet_rescale_ts(pkt, codecContext->time_base, videoStream->time_base);
            av_interleaved_write_frame(formatContext, pkt);
            av_packet_unref(pkt);
        }
    }

    av_packet_free(&pkt);
}


class CameraTestApp {
    
    public:
        CameraTestApp() {
            cameraManager = std::make_unique<CameraManager>();
            stop = 1;
        };

        ~CameraTestApp() {
            stopCamera();
        };

        bool startCamera(){
            if(stop != 1)
                return false;

            int ret;
            ret = cameraManager->start();
            if(ret){
                std::cerr << "Camera Manager could not be launched" << std::endl;
                return false;
            }

            if(cameraManager->cameras().empty()){
                std::cerr << "No Camera detected" << std::endl;
                return false; 
            }

            for (auto const &camera : cameraManager->cameras()){
                std::cout << camera->id() << std::endl;
            }

            camera = cameraManager->cameras()[0];
            camera->acquire();
            cameraConfig = camera->generateConfiguration( { StreamRole::VideoRecording } );

            streamConfig = &(cameraConfig->at(0));
            camera->requestCompleted.connect(this, &CameraTestApp::onRequestCompleted);
            stop = 0;

            return setConfig();
        };

        int allocateFrameBuffer(){
            allocator = new FrameBufferAllocator(camera);
            if(allocator->allocate(streamConfig->stream()) < 0) {
                std::cerr << "Could not allocate buffer" << std::endl;
                return -ENOMEM;
            }
            size_t allocated = allocator->buffers(streamConfig->stream()).size();
            return 0;
        }

        void capureImage(){
            Stream * stream = streamConfig->stream();
            const std::vector<std::unique_ptr<FrameBuffer>>& buffers = allocator->buffers(stream);
            std::unique_ptr<libcamera::Request> request = camera->createRequest();
            if(!request){
                std::cerr << "Could not create request" << std::endl;
                return;
            }

            if(exposure != -1 || analog_gain != -1){
                request->controls().set(controls::AeEnable, false); 
                if(exposure != -1){
                    request->controls().set(controls::ExposureTime, exposure); 
                }
                if(analog_gain != -1) {
                    request->controls().set(controls::AnalogueGain, analog_gain);
                }
            }

            request->addBuffer(stream, buffers[0].get());
            
            initFFmpeg("output.mp4");
            camera->start();
            
            


            auto startTime = std::chrono::steady_clock::now();
            auto endTime = startTime + std::chrono::seconds(durationSeconds);
            
            while (std::chrono::steady_clock::now() < endTime) {
                requestResult = false;
                camera->queueRequest(request.get());
                waitForRequest();
                if (!requestResult) {
                    std::cerr << "Request failed or cancelled\n";
                    break;
                }
                captureAndEncode(request.get());
                request->reuse(Request::ReuseBuffers);
            }
            cleanupFFmpeg();
            camera->stop();
        };
        
        void stopCamera(){
            if(camera && stop == 0){
                allocator->free(streamConfig->stream());
                delete allocator;
                camera->release();
                camera.reset();
                cameraManager->stop();
                stop = 1;
            }
        };

    private:
        std::shared_ptr<CameraManager> cameraManager;
        std::shared_ptr<Camera> camera;
        std::unique_ptr<CameraConfiguration> cameraConfig;
        StreamConfiguration * streamConfig;
        FrameBufferAllocator * allocator;
        SensorConfiguration sensorConfig;
        
        int stop;

        std::mutex mtx;
        std::condition_variable cond_variable;
        bool requestResult;

        bool setConfig(){
            
            if (hflip)
                cameraConfig->orientation = cameraConfig->orientation * libcamera::Transform::HFlip;
            if(vflip)
                cameraConfig->orientation = cameraConfig->orientation * libcamera::Transform::VFlip;
            
            
            sensorConfig.analogCrop = Rectangle(modes[mode].cropLeft, modes[mode].cropTop, modes[mode].cropWidth, modes[mode].cropHeight);
            sensorConfig.bitDepth = modes[mode].bitDepth;
            sensorConfig.binning.binX = modes[mode].binning;
            sensorConfig.binning.binY = modes[mode].binning;
            sensorConfig.skipping.xOddInc = 1;
            sensorConfig.skipping.xEvenInc = 1;
            sensorConfig.skipping.yOddInc = 1;
            sensorConfig.skipping.yEvenInc = 1;
            sensorConfig.outputSize = Size(modes[mode].width, modes[mode].height);

            if(!sensorConfig.isValid()){
                std::cerr << "Sensor configuration not available" << std::endl;
                return false;
            }
        
            cameraConfig->sensorConfig = std::optional<SensorConfiguration>(sensorConfig);

            streamConfig->size.width = width;
            streamConfig->size.height = height;
            streamConfig->pixelFormat = formats::XRGB8888;

            CameraConfiguration::Status res = cameraConfig->validate();
            if (res == CameraConfiguration::Invalid){
                std::cerr << "Configuration is not valid" << std::endl;
                return false;
            }

            camera->configure(cameraConfig.get());
            return true;
        }

        void waitForRequest() {
            std::unique_lock<std::mutex> lock(mtx);
            cond_variable.wait(lock, [this]() { return requestResult; });
        }

        void onRequestCompleted(Request * request){
            if(request->status() == Request::RequestCancelled){
                requestResult = false; 
            } else {
                requestResult = true;
            }
            cond_variable.notify_one();
        }
        
        void captureAndEncode(Request *request) {
            Stream *stream = streamConfig->stream();
            const FrameBuffer *buffer = request->buffers().at(stream);

            if (buffer->metadata().status != FrameMetadata::FrameSuccess) {
                std::cerr << "Frame capture failed\n";
                return;
            }

            const FrameBuffer::Plane &plane = buffer->planes()[0];
            void *memory = mmap(nullptr, plane.length, PROT_READ, MAP_SHARED, plane.fd.get(), 0);
            if (memory == MAP_FAILED) {
                perror("mmap failed");
                return;
            }

            encodeFrame(reinterpret_cast<uint8_t *>(memory));
            munmap(memory, plane.length);
        }

};

int imageProcessing() {
    CameraTestApp cam;
    
    if(!cam.startCamera())
        return EXIT_FAILURE;
    
    if(cam.allocateFrameBuffer() != 0)
        return EXIT_FAILURE;
    
    cam.capureImage();
    cam.stopCamera();
    
    return 0;
}

int main(int argc, char * argv[]){

    for (int i = 1; i < argc; ++i) {
        if (strcmp(argv[i], "--help") == 0) {  
            std::cout << "Usage: " << argv[0] << " [options]\n";
            std::cout   << "\t-h height of photo" << std::endl
                        << "\t-w width of photo" << std::endl
                        << "\t-V Vertical flip" << std::endl
                        << "\t-H Horizontal flip" << std::endl
                        << "\t-e exposure time"<< std::endl
                        << "\t-a analogue gain"<< std::endl
                        << "\t-m functioning mode"<< std::endl
                        << "\t-s seconds" << std::endl;
            // Add other options here
            return 0;
        }
    }

    height = 1024; 
    width = 1024;
    hflip = 0;
    vflip = 0;

    mode = 0;
    exposure = -1;
    analog_gain = -1;

    int opt;
    optind = 1;
    double exp_mult;
    while((opt = getopt(argc,argv, "h:w:VHi:j:e:m:a:s:")) != -1){
        switch(opt){
            case 'h':
                height = atoi(optarg);
                if (height <= 0) {
                    std::cerr << "Height not valid, must be positive integer greather than 0" << std::endl;
                    return EXIT_FAILURE;
                }
                break;
            case 'w':
                width = atoi(optarg);
                if (width <= 0) {
                    std::cerr << "Width not valid, must be positive integer greather than 0" << std::endl;
                    return EXIT_FAILURE;
                }
                break;
            case 'V':
                vflip = 1;
                break;
            case 'H':
                hflip = 1;
                break;
            case 'e':
                exp_mult = atof(optarg);
                exposure = exp_mult * 10000;
                if(exposure < 0) {
                    std::cerr << "Exposure is not valid, must be positive integer" << std::endl;
                    return EXIT_FAILURE;
                }
                break;
            case 'm':
                mode = atoi(optarg);
                if(mode < 0 || mode >= 4) {
                    std::cerr << "There is only four modes (0-3) available" << std::endl;
                    return EXIT_FAILURE;
                }
                break;
            case 'a':
                analog_gain = atof(optarg);
                if(analog_gain < 0) {
                    std::cerr << "Analog Gain is not valid, must be positive integer" << std::endl;
                    return EXIT_FAILURE;
                }
                break;
            case 's':
                durationSeconds = atoi(optarg);
                if (durationSeconds <= 0) {
                    std::cerr << "durationSeconds not valid, must be positive integer greather than 0" << std::endl;
                    return EXIT_FAILURE;
                }
                break;
        }
    }


    int ret = imageProcessing();
    return ret;
}
