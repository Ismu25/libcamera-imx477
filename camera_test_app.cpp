#include <iomanip>
#include <unistd.h>
#include <iostream>
#include <memory>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <sys/mman.h>
#include <syslog.h>

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
};

static const mode_struct modes[] = {
        {.bitDepth= 12, .width= 4056, .height= 3040, .binning = 1, .cropLeft =   8, .cropTop =  16, .cropWidth = 4056, .cropHeight = 3040},
        {.bitDepth= 12, .width= 2028, .height= 1520, .binning = 2, .cropLeft =   8, .cropTop =  16, .cropWidth = 4056, .cropHeight = 3040},
        {.bitDepth= 12, .width= 2028, .height= 1080, .binning = 2, .cropLeft =   8, .cropTop = 456, .cropWidth = 4056, .cropHeight = 2160},
        {.bitDepth= 10, .width= 1332, .height= 990 , .binning = 2, .cropLeft = 704, .cropTop = 544, .cropWidth = 2664, .cropHeight = 1980}
    
};
 
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
            cameraConfig = camera->generateConfiguration( { StreamRole::StillCapture } );
            for(auto it = cameraConfig->begin(); it != cameraConfig->end(); it++){
                std::cout << (*it).toString() << std::endl;
            }

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

        void capureImage(const std::string &filePath){
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
            camera->start();
            camera->queueRequest(request.get());
            waitForRequest();

            processBuffer(buffers[0].get());
            camera->stop();
        }
        
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

        void onRequestCompleted(Request * request){
            if(request->status() == Request::RequestCancelled){
                requestResult = false; 
            } else {
                requestResult = true;
            }
            cond_variable.notify_one();
        }

        void waitForRequest() {
            std::unique_lock<std::mutex> lock(mtx);
            cond_variable.wait(lock, [this]() { return requestResult; });
        }

        void processBuffer(FrameBuffer * buffer) {
           // Access the first plane of the FrameBuffer
            FrameBuffer::Plane plane = buffer->planes()[0];
            
            // Get the DMA-BUF file descriptor
            int fd = plane.fd.get();
            
            // Get the length of the plane data
            size_t length = plane.length;
            size_t offset = plane.offset;
            // Map the DMA-BUF to memory at the correct offset
            uint8_t *mappedData = static_cast<uint8_t *>(mmap(NULL, length, PROT_READ | PROT_WRITE, MAP_SHARED, fd, offset));
            if (mappedData == MAP_FAILED) {
                std::cerr << "Failed to mmap buffer" << std::endl;
                return;
            }
            // Allocate a contiguous buffer for the image data
            size_t rowSize = width * 4; // Assuming 4 bytes per pixel (XRGB8888)

            int numberUnsuedBytes = (0x40 - (rowSize % 0x40)) % 0x40 ;
            size_t paddedRowSize = rowSize + numberUnsuedBytes; // Padded row size
            uint8_t *contiguousData = new uint8_t[height * rowSize];
            
            // Copy data from the padded buffer to the contiguous buffer
            for (int y = 0; y < height; ++y) {
                std::memcpy(contiguousData + y * rowSize, mappedData + y * paddedRowSize, rowSize);
            }

            try {
                // Create a cv::Mat object from the contiguous data
                cv::Mat img(height, width, CV_8UC4, contiguousData);

                // Save the image as PNG
                if (!cv::imwrite("output_image.png", img)) {
                    std::cerr << "Failed to write image file" << std::endl;
                }
            } catch (const cv::Exception &e) {
                std::cerr << "OpenCV exception: " << e.what() << std::endl;
            }

            // Clean up
            delete[] contiguousData;

            // Unmap the memory
            if (munmap(mappedData, length) == -1) {
                std::cerr << "Failed to unmap buffer" << std::endl;
            }
        }

};

int imageProcessing() {
    CameraTestApp cam;
    if(!cam.startCamera())
        return EXIT_FAILURE;
    if(cam.allocateFrameBuffer() != 0)
        return EXIT_FAILURE;
    cam.capureImage("test.jpeg");
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
                        << "\t-m functioning mode"<< std::endl;
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
    while((opt = getopt(argc,argv, "h:w:VHi:j:e:m:a:")) != -1){
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
        }
    }


    openlog("imx477-client", LOG_PID | LOG_CONS, LOG_USER);
    int ret = imageProcessing();
    closelog();
    return ret;
}
