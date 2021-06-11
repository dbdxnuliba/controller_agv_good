#include <iostream>
#include <rest_rpc.hpp>
#include <chrono>
#include <fstream>
#include <event2/event.h>


using namespace rest_rpc;
using namespace rest_rpc::rpc_service;
struct person {
	int id;
	std::string name;
	int age;

	MSGPACK_DEFINE(id, name, age);
};
static rpc_client GLOBAL_CLIENT("127.0.0.1", 9000);

//using namespace msgpack;

namespace aaaaa 
{
	enum E_A
	{
		E_1 = 0,
		E_2,

	};
	//MSGPACK_ADD_ENUM(E_A);
};


MSGPACK_ADD_ENUM(aaaaa::E_A);

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


typedef struct {
    struct timeval tv;
    struct event * ev;
} timer_param_t;

inline int64_t time_stamp_us()
{
    std::chrono::steady_clock::time_point tp = std::chrono::steady_clock::now();
    std::chrono::microseconds us = std::chrono::duration_cast<std::chrono::microseconds>(tp.time_since_epoch());
    return (int64_t) us.count();
}

void test_hello1(const char* data) {
	try {
		rpc_client client("127.0.0.1", 9000);
		printf("start calling hello1 %.3f s\n", time_stamp_us() * 1e-6);
		//std::cout << "start calling hello1 " << time_stamp_us() * 0.001  << std::endl;
		bool r = client.connect(100);
		if (!r) {
			std::cout << "test_hello1 connect timeout" << std::endl;
			return;
		}
		
		client.call("hello1", data);
	}
	catch (const std::exception & e) {
		std::cout << "hello1 exception:" << e.what() << std::endl;
	}
}

void test_hello2(const char* data) {
	try {
		rpc_client client("127.0.0.1", 9000);
		printf("start calling hello2 %.3f s\n", time_stamp_us() * 1e-6);
		bool r = client.connect(0);
		if (!r) {
			std::cout << "test_hello2 connect timeout" << std::endl;
			return;
		}

		client.call<0>("hello2", data);
	}
	catch (const std::exception & e) {
		std::cout << "hello2 exception:"  << e.what() << std::endl;
	}
}

void test_add(int a, int b) {
	try {
		rpc_client client("127.0.0.1", 9000);
		printf("start calling hello1 %.3f s\n", time_stamp_us() * 1e-6);
		//std::cout << "start calling hello1 " << time_stamp_us() * 0.001  << std::endl;
		bool r = client.connect(100);
		if (!r) {
			std::cout << "test_hello1 connect timeout" << std::endl;
			return;
		}
		// int ret;
		// client.call("hello1", a, b, ret);
		// printf(" %d + %d = %d\n", a, b, ret);
	}
	catch (const std::exception & e) {
		std::cout << "hello1 exception:" << e.what() << std::endl;
	}
}

void test_async_client_hello1(const char* data) {
	static rpc_client client("127.0.0.1", 9000);
	bool r = client.connect(200);
	if (!r) {
		std::cout << "test_async_client_hello1 connect timeout" << std::endl;
		return;
	}

	client.set_error_callback([](boost::system::error_code ec) {
		std::cout << ec.message() << std::endl;
	});

	//auto f = client.async_call<FUTURE>("get_person");
	// if (f.wait_for(std::chrono::milliseconds(50)) == std::future_status::timeout) {
	// 	std::cout << "timeout" << std::endl;
	// }
	// else {
	// 	auto p = f.get().as();
	// 	//std::cout << p.name << std::endl;
	// }

	//auto fu = client.async_call<FUTURE>("hello1", data);
	//fu.get().as(); //no return

	// //sync call
	// client.call("hello", "purecpp");
	// auto p = client.call<person>("get_person");

	// std::string str;
	// std::cin >> str;
	client.async_call<0>("hello1", [](boost::system::error_code ec, string_view sv) 
	{
	  if (ec) {
		  std::cout << ec.message() <<" "<< sv << "\n";
		  printf("hello1() call back error!!!!!!!!!\n");
        return;
      }
	  std::cout <<"sv: "<< sv << "\n";
		printf("hello1() call back finished\n\n");
    }, data);
}

void timer_task1(int fd, short events, void * ctx) 
{
	timer_param_t * param = (timer_param_t *) ctx;
    evtimer_add(param->ev, &param->tv);

    printf("timer_task1 ...\n");
    printf("timer_task1 time stamp %.3f\n",  time_stamp_us() * 1e-6);
	char data[10];
	sprintf(data, "%.3f", time_stamp_us() * 1e-6);
    //std::this_thread::sleep_for(std::chrono::seconds(10));


	rpc_client client("127.0.0.1", 9000);
	//client.enable_auto_reconnect();
	//client.async_connect("127.0.0.1", 9000);
	bool r = client.connect(2);
	if (!r) {
		std::cout << "test_async_client_hello1 connect timeout" << std::endl;
		return;
	}

	// auto fu = client.async_call<FUTURE>("hello1", data);
	// fu.get().as(); //no return
	client.async_call<0>("hello1", [](boost::system::error_code ec, string_view sv) 
	{
	  if (ec) {
		  std::cout << ec.message() << "\n";
		  printf("hello1() call back error!!!!!!!!!\n");
        return;
      }
	  auto str = as<std::string>(sv);

	  std::cout <<"sv: "<< sv << "\n";
		printf("hello1() call back finished\n\n");
    }, data);
	
	//printf("timer_task1 finished\n");
}

void timer_task2(int fd, short events, void * ctx) {
	timer_param_t * param = (timer_param_t *) ctx;
    evtimer_add(param->ev, &param->tv);
    printf("timer_task2 ...\n");
    printf("timer_task2 time stamp %.3f\n",  time_stamp_us() * 1e-6);
    //std::this_thread::sleep_for(std::chrono::seconds(10));
}

void test_sub1() {
	rpc_client client;
	client.enable_auto_reconnect();
	client.enable_auto_heartbeat();
	bool r = client.connect("127.0.0.1", 9000);
	if (!r) {
		return;
	}

	client.subscribe("key", [](string_view data) {
		std::cout << data << "\n";
	});

	// client.subscribe("key", "048a796c8a3c6a6b7bd1223bf2c8cee05232e927b521984ba417cb2fca6df9d1", [](string_view data) {
	// 	msgpack_codec codec;
	// 	person p = codec.unpack<person>(data.data(), data.size());
	// 	std::cout << p.name << "\n";
	// });

	// client.subscribe("key1", "048a796c8a3c6a6b7bd1223bf2c8cee05232e927b521984ba417cb2fca6df9d1", [](string_view data) {
	// 	std::cout << data << "\n";
	// });

	// bool stop = false;
	// std::thread thd1([&client, &stop] {
	// 	while (true) {
	// 		try {
	// 			if (client.has_connected()) {
	// 				//int r = client.call<int>("add", 2, 3);
	// 				//std::cout << "add result: " << r << "\n";
	// 			}
				
	// 			std::this_thread::sleep_for(std::chrono::milliseconds(100));
	// 		}
	// 		catch (const std::exception& ex) {
	// 			std::cout << ex.what() << "\n";
	// 		}
	// 	}
	// });

	/*rpc_client client1;
	bool r1 = client1.connect("127.0.0.1", 9000);
	if (!r1) {
		return;
	}

	person p{10, "jack", 21};
	client1.publish("key", "hello subscriber");
	client1.publish_by_token("key", "sub_key", p);
	
	std::thread thd([&client1, p] {
		while (true) {
			try {
				client1.publish("key", "hello subscriber");
				client1.publish_by_token("key", "unique_token", p);				
			}
			catch (const std::exception& ex) {
				std::cout << ex.what() << "\n";
			}
		}
	});
*/

	std::string str;
	std::cin >> str;
}

int main() 
{
	//test_sub1();
	


#if 1
	int i = 0;
	char data[10];
	//test_async_client_hello1("0");
	static rpc_client client("127.0.0.1", 9000);
	bool r = client.connect(0);
	if (!r) {
		std::cout << "client.connect(0) connect timeout" << std::endl;
		//return;
	}
	//test_sub();
		//std::getchar();
	while(++i < 10)
	{
		sprintf(data, "%d", i);
		//test_add(i, i * 2);
		//test_hello1(data);
		//test_async_client_hello1(data);
			
		// client.async_call<0>("hello1", [i](boost::system::error_code ec, string_view sv) {
		
		// if (ec) {
		// 	std::cout << ec.message() <<" "<< sv << "\n";
		// 	printf("hello1(%d) call back error!!!!!!!!!\n", i);
		// 	return;
		// }
		// 	printf("hello1(%d) call back finished\n\n", i);
		// }, data);
		printf("call hello1 %s\n", data);
		//client.call_without_timeout("hello1", data);
		client.call("hello1", data);
		printf("call hello1 %s\n finished", data);
		std::this_thread::sleep_for(std::chrono::seconds(1));
		//test_hello2(data);
	}

#endif
#if 0
	struct event_base * base;
    base = event_base_new();
    if(!base) {
        printf("open event base error\n");
        return -1;
    }
    timer_param_t * param = (timer_param_t*)calloc(1, sizeof(timer_param_t));
    param->ev = evtimer_new(base, timer_task1, param);
    param->tv.tv_sec = 1;
    evtimer_add(param->ev, &param->tv);

    timer_param_t * param2 = (timer_param_t*)calloc(1, sizeof(timer_param_t));
    param2->ev = evtimer_new(base, timer_task2, param2);
    param2->tv.tv_sec = 0;
    param2->tv.tv_usec = 500*1000;
    evtimer_add(param2->ev, &param2->tv);

    int err = event_base_dispatch(base);
    printf("error = %d\n", err);
#endif
	std::getchar();
	std::cout<<"client exist\n";
	return 0;
}