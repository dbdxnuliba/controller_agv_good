#include <rest_rpc.hpp>
using namespace rest_rpc;
using namespace rpc_service;
#include <fstream>

#include "qps.h"

class A
{
public:
	int id;
	std::vector<int> list;
	MSGPACK_DEFINE(id, list);
};

template <typename T>
class AA
{
public:
	T data;
	A c_a;
	MSGPACK_DEFINE(data, c_a);
};

static std::mutex hello1_mtx;

int64_t time_stamp_us()
{
    std::chrono::steady_clock::time_point tp = std::chrono::steady_clock::now();
    std::chrono::microseconds us = std::chrono::duration_cast<std::chrono::microseconds>(tp.time_since_epoch());
    return (int64_t) us.count();
};

void hello1(rpc_conn conn, const std::string& str) {
	//std::cout << "recv hello1 calling " << time_stamp_us() * 0.001 << std::endl;
	printf("----------------recv calling hello1(%s) %.3f s\n", str.c_str(), time_stamp_us() * 1e-6);
	if(!hello1_mtx.try_lock())
	{
		std::cout << "hello1 is running return failed" << std::endl;
		return;
	}
	
	std::cout << "hello1 " << str << std::endl;
	std::cout << "task id: " << std::this_thread::get_id() << std::endl;
	std::this_thread::sleep_for(std::chrono::seconds(10));
	std::cout << "hello1 finished deal" << str << std::endl;
	hello1_mtx.unlock();
}

void hello2(rpc_conn conn, const std::string& str) {
	printf("+++++++++++++++++recv calling hello2(%s) %.3f s\n", str.c_str(), time_stamp_us() * 1e-6);
	std::cout << "hello2 " << str << std::endl;
}



void server1()
{
	//rpc_server server(9000, std::thread::hardware_concurrency(), 0, 0);
	rpc_server server(9000, 4, 0, 0);


	server.register_handler("hello1", hello1);
	
	server.run();
}

void server2()
{
	rpc_server server(9001, std::thread::hardware_concurrency());


	server.register_handler("hello2", hello2);
	
	server.run();
}
int main() {
//  benchmark_test();

	// std::thread t1(server1);
	// t1.detach();
	// std::thread t2(server2);
	// t2.detach();
	//rpc_server server(9000, 4, 0, 0);
	rpc_server server(9000, std::thread::hardware_concurrency(), 1000, 1000);
	server.register_handler("hello1", hello1);
	server.register_handler("hello2", hello2);


	// server.register_handler("publish_by_token", 
	// [&server](rpc_conn conn, std::string key, std::string token, std::string val) {
	// 	server.publish_by_token(std::move(key), std::move(token), std::move(val));}
	// );

	// server.register_handler("publish", 
	// [&server](rpc_conn conn, std::string key, std::string token, std::string val) {
	// 	server.publish(std::move(key), std::move(val));}
	// );

	std::thread thd([&server] {
		//person p{ 1, "tom", 20 };
		int i = 0;
		char data[100];
		while (true) {
			sprintf(data, "publish key %d %.3fs", i, time_stamp_us() * 1e-6);
			++i;
			//printf("server publish\n");
			server.publish("key", data);
			auto list = server.get_token_list();
			for (auto& token : list) {				
				server.publish_by_token("key", token, "publish_by_token key");
				server.publish_by_token("key1", token, "publish_by_token key1");
			}
			std::this_thread::sleep_for(std::chrono::milliseconds(500));
		}
	});

	server.run();

	// std::string str;
	// std::cin >> str;
}