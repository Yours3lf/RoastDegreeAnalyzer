#include "libwebsocket/WebsocketServer.h"

#include "ImageRoaster/ImageRoaster.h"

#include <libcamera/libcamera.h>

#include <iomanip>
#include <unistd.h>
#include <sys/mman.h>

using namespace libcamera;
static std::shared_ptr<Camera> camera;
std::unique_ptr<websocketServer> ws;

std::unordered_map<int, std::pair<void*, size_t> > fdToMem;
std::unordered_map<int, std::pair<size_t, size_t> > fdToMapInfo;

static ImageRoaster ic;

std::vector<uint8_t> threadDataA;
uint32_t threadDataTypeA = 0, threadDataWidthA = 0, threadDataHeightA = 0;
std::vector<uint8_t> threadDataB;
uint32_t threadDataTypeB = 0, threadDataWidthB = 0, threadDataHeightB = 0;
bool threadDataAorB = true;
std::mutex threadDataSelectorMutex;
std::mutex threadDataMutexA;
std::mutex threadDataMutexB;

void compressAndSend(uint8_t* data, uint32_t type, uint32_t width, uint32_t height)
{
	std::unique_ptr<websocketMessage> m(new websocketMessage());
	m->type = FRAME_BINARY;

	if(type == 1)
	{
		std::vector<uint8_t> resBuf;
		ic.compressImage<uint8_t>(resBuf, data, 8, 4, width, height, 8);
		m->buf.resize(sizeof(uint32_t) + resBuf.size());
		*(uint32_t*)(m->buf.data()) = type;
		memcpy(m->buf.data() + sizeof(uint32_t), resBuf.data(), resBuf.size());
	}
	else if(type == 2)
	{
		const uint16_t* pixels = (const uint16_t*)data;

		uint16_t maxVal = 0x3ff;

		uint32_t counter = 0;

		//process raw image row-by-row
		//each row will contain either R-G or B-G components alternating
		//eg.
		//BGBGBGBG
		//GRGRGRGR
		//BGBGBGBG
		//GRGRGRGR
		//this would normally get de-bayered, but we'll just extract 
		//the red component and work with that

		auto start = std::chrono::system_clock::now();

		std::vector<uint16_t> rChannel;
		rChannel.resize((width >> 1) * (height >> 1));

		//skip even rows, we're only interested in R values
		for(int y = 1; y < height; y += 2)
		{
			//skip even columns, we're only interested in R values
			for(int x = 1; x < width; x += 2)
			{
				//extract bottom 10bits
				uint16_t pixel = pixels[y * width + x] & maxVal; //

				rChannel[counter++] = pixel;
			}
		}

		auto end = std::chrono::system_clock::now();
		auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
		std::cout << "R channel extraction time: " << elapsed << std::endl;

		std::vector<uint8_t> resBuf;
		ic.compressImage<uint16_t>(resBuf, rChannel.data(), 10, 1, width >> 1, height >> 1, 8);
		m->buf.resize(sizeof(uint32_t) + resBuf.size());
		*(uint32_t*)(m->buf.data()) = type;
		memcpy(m->buf.data() + sizeof(uint32_t), resBuf.data(), resBuf.size());
	}

	ws->broadcastMessage(std::move(m));
}

void compressionThreadFunc()
{
	while(ws->hasConnections())
	{
		uint32_t width, height, type;
		uint8_t* data;
		{
			std::lock_guard<std::mutex> guard(threadDataSelectorMutex);	

			if(threadDataAorB)
			{
				data = threadDataA.data();
				width = threadDataWidthA;
				height = threadDataHeightA;
				type = threadDataTypeA;
				threadDataMutexA.lock();
			}
			else
			{
				data = threadDataB.data();
				width = threadDataWidthB;
				height = threadDataHeightB;
				type = threadDataTypeB;
				threadDataMutexB.lock();
			}
		}

		if(width == 0 || height == 0 || type == 0 || data == 0)
		{
			{
				std::lock_guard<std::mutex> guard(threadDataSelectorMutex);	
	
				if(threadDataAorB)
				{
					threadDataMutexA.unlock();
					threadDataAorB = !threadDataAorB;
				}
				else
				{
					threadDataMutexB.unlock();
					threadDataAorB = !threadDataAorB;
				}
			}

			std::this_thread::sleep_for(std::chrono::microseconds(1000));
			continue;
		}

		compressAndSend(data, type, width, height);

		{
			std::lock_guard<std::mutex> guard(threadDataSelectorMutex);	

			if(threadDataAorB)
			{
				threadDataTypeA = 0;
				threadDataMutexA.unlock();
				threadDataAorB = !threadDataAorB;
			}
			else
			{
				threadDataTypeB = 0;
				threadDataMutexB.unlock();
				threadDataAorB = !threadDataAorB;
			}
		}
	}
}

static void requestComplete(Request *request)
{
    if (request->status() == Request::RequestCancelled)
	{
		return;
	}

	const std::map<const Stream*, FrameBuffer*>& buffers = request->buffers();

	/*
	std::cout << "#buffers: " << buffers.size() << std::endl;

	for (auto bufferPair : buffers)
	{
		FrameBuffer* buffer = bufferPair.second;
		const FrameMetadata& metadata = buffer->metadata();

		std::cout << "#planes: " << metadata.planes().size() << std::endl;

		std::cout << " seq: " << std::setw(6) << std::setfill('0') << metadata.sequence << " bytesused: ";

		unsigned int nplane = 0;
		for (const FrameMetadata::Plane& plane : metadata.planes())
		{
			std::cout << plane.bytesused;
			if (++nplane < metadata.planes().size()) std::cout << "/";
		}

		std::cout << std::endl;
	}
	*/

	FrameBuffer* buffer = buffers.begin()->second;
	const FrameBuffer::Plane& plane = *buffer->planes().begin();
	const FrameMetadata& metadata = buffer->metadata();
	const FrameMetadata::Plane& planeMetaData = *metadata.planes().begin();
	uint32_t stride = buffers.begin()->first->configuration().stride;
	auto dimensions = buffers.begin()->first->configuration().size;

	uint32_t type = buffers.begin()->first->configuration().colorSpace == ColorSpace::Srgb ? 1 : 2;

	bool dataWritten = false;
	while(!dataWritten)
	{
		{
			std::lock_guard<std::mutex> guard(threadDataSelectorMutex);	

			if(threadDataAorB)
			{
				std::lock_guard<std::mutex> guard(threadDataMutexB);
				if(!threadDataTypeB) //only write data if the previous data has been consumed
				{
					threadDataB.resize(std::max(planeMetaData.bytesused, (unsigned int)threadDataB.size()));
					memcpy(threadDataB.data(), fdToMem[plane.fd.get()].first, planeMetaData.bytesused);
					threadDataWidthB = dimensions.width;
					threadDataHeightB = dimensions.height;
					threadDataTypeB = type;
					dataWritten = true;
				}
			}
			else
			{
				std::lock_guard<std::mutex> guard(threadDataMutexA);
				if(!threadDataTypeA)
				{
					threadDataA.resize(std::max(planeMetaData.bytesused, (unsigned int)threadDataA.size()));
					memcpy(threadDataA.data(), fdToMem[plane.fd.get()].first, planeMetaData.bytesused);
					threadDataWidthA = dimensions.width;
					threadDataHeightA = dimensions.height;
					threadDataTypeA = type;
					dataWritten = true;
				}
			}
		}

		std::this_thread::sleep_for(std::chrono::microseconds(1000));
	}

	//for debugging, make it single threaded
	//compressAndSend((uint8_t*)fdToMem[plane.fd.get()].first, type, dimensions.width, dimensions.height);

	//std::cout << "Seq#: " << metadata.sequence << std::endl;
	//std::cout << "Size: " << planeMetaData.bytesused << std::endl;

	//continually re-request frames for the viewfinder
	if(type == 1)
	{
		request->reuse(Request::ReuseBuffers);
		camera->queueRequest(request);
	}
}

void addRequestsForStream(Stream* stream, FrameBufferAllocator* allocator, std::vector<std::unique_ptr<Request>>& requests)
{
	if(!stream) return;
	if(!allocator) return;
	
	const std::vector<std::unique_ptr<FrameBuffer>>& buffers = allocator->buffers(stream);

	for (unsigned int i = 0; i < buffers.size(); ++i) 
	{
		std::unique_ptr<Request> request = camera->createRequest();
		if (!request)
		{
			std::cerr << "Can't create request" << std::endl;
			return;
		}
	
		const std::unique_ptr<FrameBuffer>& buffer = buffers[i];
		if (request->addBuffer(stream, buffer.get()) < 0)
		{
			std::cerr << "Can't set buffer for request" << std::endl;
			return;
		}
	
		requests.push_back(std::move(request));
	}
}

void enqueueRequests(const std::vector<std::unique_ptr<Request>>& requests)
{
	for (const std::unique_ptr<Request>& request : requests)
	{
		if(request->status() == Request::Status::RequestPending)
		{
			camera->queueRequest(request.get());
		}
	}
}

int main(int argc, char** args)
{
	if (argc < 2)
	{
		std::cerr << "Usage " << args[0] << " exposureTime(us) address port" << std::endl;
		std::cerr << "Default 10000us and port 50000 used if omitted" << std::endl;
	}

	int exposureTime = 10000;
	if(argc > 1)
	{
		exposureTime = std::stoi(std::string(args[1]));
	}

	std::string address;
	if(argc > 2)
	{
		address = std::string(args[2]);
	}

	int port = 50000;
	if (argc > 3)
	{
		port = std::stoi(std::string(args[3]));
	}

	std::cout << "Exposure time: " << exposureTime << " us" << std::endl;
	std::cout << "Listening address: " << address << ":" << port << std::endl;

	socket::init();

	ws = std::make_unique<websocketServer>();

	///////////////
	//init camera

	std::unique_ptr<CameraManager> cm = std::make_unique<CameraManager>();
	cm->start();

	//enumerate cameras
	auto cameras = cm->cameras();
	for(auto const& cam : cameras)
	{
		std::cout << "Camera found: " << cam->id() << std::endl;
	}

	if(cameras.empty())
	{
		std::cerr << "No cameras found." << std::endl;
		cm->stop();
		return -1;
	}

	//pick the first, and get the actual camera
	std::string camID = cameras[0]->id();
	camera = cm->get(camID);

	//acquire lock on camera so no other app can use it
	camera->acquire();

	//genereate ViewFinder config
	std::unique_ptr<CameraConfiguration> config = camera->generateConfiguration( { StreamRole::Viewfinder, StreamRole::StillCapture } );

	StreamConfiguration& viewFinderConfig = config->at(0);
	std::cout << "Default viewfinder configuration is: " << viewFinderConfig.toString() << std::endl;

	uint32_t camWidthPixels = 4608;
	uint32_t camHeightPixels = 2592;

	//adjust first stream config, must validate after
	viewFinderConfig.size.width = camWidthPixels >> 2;
	viewFinderConfig.size.height = camHeightPixels >> 2;
	viewFinderConfig.colorSpace = ColorSpace::Srgb;
	viewFinderConfig.pixelFormat = formats::XRGB8888;

	//create square still capture config
	StreamConfiguration& stillCaptureConfig = config->at(1);
	stillCaptureConfig.size.width = camWidthPixels;
	stillCaptureConfig.size.height = camHeightPixels;
	stillCaptureConfig.colorSpace = ColorSpace::Raw;
	stillCaptureConfig.pixelFormat = formats::SBGGR10;

	config->validate();

	//check config after, it could be different
	std::cout << "Validated viewfinder configuration is: " << viewFinderConfig.toString() << std::endl;
	std::cout << "Validated still capture configuration is: " << stillCaptureConfig.toString() << std::endl;

	//apply configuration to the camera
	camera->configure(config.get());

	//allocate framebuffer for the camera to use
	FrameBufferAllocator* allocator = new FrameBufferAllocator(camera);

	for (StreamConfiguration& cfg : *config) 
	{
		if (allocator->allocate(cfg.stream()) < 0) 
		{
			std::cerr << "Can't allocate buffers" << std::endl;
			return -1;
		}
	
		size_t allocated = allocator->buffers(cfg.stream()).size();
		std::cout << "Allocated " << allocated << " buffers for stream" << std::endl;

		for(auto& buf : allocator->buffers(cfg.stream()))
		{
			for(auto& plane : buf->planes())
			{
				int fd = plane.fd.get();
				auto& info = fdToMapInfo[fd];
				if(!info.second)
				{
					auto totalLen = lseek64(fd, 0, SEEK_END);
					info = std::make_pair(0, totalLen);
				}

				if(plane.offset + plane.length > info.second)
				{
					std::cerr << "plane out of bounds" << std::endl;
					return -1;
				}

				info.first = std::max(info.first, std::size_t(plane.offset + plane.length));
			}
		}
	}

	for(auto& mapInfo : fdToMapInfo)
	{
		void* ptr = mmap64(0, mapInfo.second.first, PROT_READ, MAP_PRIVATE, mapInfo.first, 0);

		if(!ptr)
		{
			std::cerr << "mmap64 failed" << std::endl;
			return -1;
		}

		fdToMem[mapInfo.first] = std::make_pair(ptr, mapInfo.second.first);
	}

	//Capture frames using request
	std::vector<std::unique_ptr<Request>> requests;
	for (StreamConfiguration& cfg : *config) 
	{
		addRequestsForStream(cfg.stream(), allocator, requests);
	}

	ws->run(address, port);

	camera->requestCompleted.connect(requestComplete);

	//TODO disable auto exposure
	//set exposure manually

	//set up camera controls
	ControlList	controlList;
	controlList.set(controls::AeEnable, false);
	//controlList.set(controls::AeMeteringMode, controls::MeteringCentreWeighted);
	//controlList.set(controls::AeConstraintMode, controls::ConstraintHighlight);
	//controlList.set(controls::AeExposureMode, ExposureNormal);
	//controlList.set(controls::AeFlickerMode, controls::FlickerOff);
	//controlList.set(controls::ExposureValue, true);
	controlList.set(controls::ExposureTime, exposureTime); //microseconds
	//controlList.set(controls::ExposureTimeMode, true);
	controlList.set(controls::AnalogueGain, 1.0f); //float, must be >=1.0
	//controlList.set(controls::AnalogueGainMode, true);
	controlList.set(controls::AfMode, controls::AfModeContinuous);
	//controlList.set(controls::AfMetering, true);

	//start camera, request queuing
	camera->start(&controlList);
	enqueueRequests(requests);

	//wait for a connection
	while (!ws->hasConnections())
	{
		std::this_thread::sleep_for(std::chrono::milliseconds(10));
	}

	std::thread compressionThread(compressionThreadFunc);

	while (ws->hasConnections())
	{
		auto start = std::chrono::system_clock::now();

		while(ws->hasMessagesReceived())
		{
			auto m = ws->popMessageReceived();

			if (!m.second) continue;

			m.second->buf.push_back('\0');

			if(m.second->type == FRAME_TEXT)
			{
				std::string str = std::string(m.second->buf.data());

				std::cout << "Message received: " << str << std::endl;

				if(std::string("capture").compare(str) == 0)
				{
					addRequestsForStream(stillCaptureConfig.stream(), allocator, requests);

					enqueueRequests(requests);
				}
			}
			else if (m.second->type == FRAME_CLOSE)
			{
				std::cout << "Close frame received" << std::endl;
				break;
			}
		}

#ifdef _WIN32
		std::this_thread::sleep_for(std::chrono::microseconds(1000));
#else
		auto end = std::chrono::system_clock::now();
		auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
		auto offset_usec = std::chrono::microseconds(850); //offset added to make sure frames are ready in time
		//auto frame_time = std::chrono::microseconds(10000); 
		//std::this_thread::sleep_for(std::chrono::microseconds(frame_time - elapsed + offset_usec));
		std::this_thread::sleep_for(std::chrono::microseconds(offset_usec));
#endif
	}

	compressionThread.join();

	ws->close();

	socket::shutdown();

	//cleanup
	camera->stop();
	for (StreamConfiguration& cfg : *config) 
	{
		allocator->free(cfg.stream());
	}
	delete allocator;
	camera->release();
	camera.reset();
	cm->stop();

	return 0;
}
