#include "libwebsocket/WebsocketServer.h"

#include <libcamera/libcamera.h>

#include <cmath>

#include <stdlib.h>
#include <sys/wait.h>
#include <fstream>

using namespace libcamera;
static std::shared_ptr<Camera> camera;

int main(int argc, char** args)
{
	if (argc < 2)
	{
		std::cerr << "Usage " << args[0] << " address port" << std::endl;
		std::cerr << "Default port 50000 used if omitted" << std::endl;
	}

	std::string address;
	if(argc > 1)
	{
		address = std::string(args[1]);
	}

	int port = 50000;
	if (argc > 2)
	{
		port = std::stoi(std::string(args[2]));
	}

	socket::init();

	{
		websocketServer ws;
		ws.run(address, port);

		//wait for a connection
		while (!ws.hasConnections())
		{
			std::this_thread::sleep_for(std::chrono::milliseconds(10));
		}

		while (ws.hasConnections())
		{
			auto start = std::chrono::system_clock::now();

			while(ws.hasMessagesReceived())
			{
				auto m = ws.popMessageReceived();

				if (!m.second) continue;

				m.second->buf.push_back('\0');
				std::cout << "Message received: " << m.second->buf.data() << std::endl;

				if (m.second->type == FRAME_CLOSE)
				{
					ws.close();
					break;
				}
			}

#ifdef _WIN32
			std::this_thread::sleep_for(std::chrono::microseconds(1000));
#else
			auto end = std::chrono::system_clock::now();
            auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
            auto offset_usec = std::chrono::microseconds(850); //offset added to make sure frames are ready in time
            //std::this_thread::sleep_for(std::chrono::microseconds(frame_time - elapsed + offset_usec));
			std::this_thread::sleep_for(std::chrono::microseconds(1000));
#endif
		}
	}

	socket::shutdown();

	return 0;
}
