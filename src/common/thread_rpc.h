/*
 * 该方法只能用于在同一进程中的线程间通信
 * 设计目的：
 * 实际应用中，各功能模块可以不止一个线程，该方法是为了各模块间相互通信而设计，实现N:M
 */

#pragma once
#ifndef __THREAD_RPC__
#define __THREAD_RPC__

#include <iostream>
#include <thread>
#include <queue>
#include <atomic>
#include <condition_variable>
#include <mutex>
#include <unordered_map>
#include <stdio.h>
#include <string>
#include <deque>
#include <future>
#include <utility>
#include <assert.h>
#include <functional>
#include <iterator>
#include <algorithm>
#include <typeinfo>
#include <iomanip>
#include <tuple>
#include <type_traits>
#include <memory>
#include "common/print.h"
//#include "cpp_14_plugins.h"

#define __USE_SEM__ 1

#ifdef __USE_SEM__
#include <semaphore.h>
#endif

//-----------------------
// start of cpp_14_plugins.h

#ifndef CPP_14_PLUGINS_H
#define CPP_14_PLUGINS_H

#ifndef REST_RPC_CPLUSPLUS_14_H_
#define REST_RPC_CPLUSPLUS_14_H_

//		#include <type_traits>
//		#include <memory>
//		#include <tuple>

#if __cplusplus == 201103L

namespace std {
template<class T>
struct unique_if {
  typedef unique_ptr<T> single_object;
};

template<class T>
struct unique_if<T[]> {
  typedef unique_ptr<T[]> unknown_bound;
};

template<class T, size_t N>
struct unique_if<T[N]> {
  typedef void known_bound;
};

template<class T, class... Args>
typename unique_if<T>::single_object make_unique(Args&&... args) {
  return unique_ptr<T>(new T(forward<Args>(args)...));
}

template<class T>
typename unique_if<T>::unknown_bound make_unique(size_t n) {
  typedef typename remove_extent<T>::type U;
  return unique_ptr<T>(new U[n]());
}

template<class T, class... Args>
typename unique_if<T>::known_bound make_unique(Args&&...) = delete;

template<size_t... Ints>
struct index_sequence {
  using type = index_sequence;
  using value_type = size_t;
  static constexpr std::size_t size() noexcept { return sizeof...(Ints); }
};

// --------------------------------------------------------------

template<class Sequence1, class Sequence2>
struct _merge_and_renumber;

template<size_t... I1, size_t... I2>
struct _merge_and_renumber<index_sequence<I1...>, index_sequence<I2...>>
    : index_sequence<I1..., (sizeof...(I1) + I2)...> {};

// --------------------------------------------------------------

template<size_t N>
struct make_index_sequence : _merge_and_renumber<typename make_index_sequence<N / 2>::type,
                                                 typename make_index_sequence<N - N / 2>::type> {};

template<>
struct make_index_sequence<0> : index_sequence<> {};
template<>
struct make_index_sequence<1> : index_sequence<0> {};

template<typename... T>
using index_sequence_for = make_index_sequence<sizeof...(T)>;

template<bool B, class T = void>
using enable_if_t = typename enable_if<B, T>::type;

template<typename T>
using remove_const_t = typename remove_const<T>::type;

template<typename T>
using remove_reference_t = typename remove_reference<T>::type;

template<int I, typename T>
using tuple_element_t = typename tuple_element<I, T>::type;

template<typename T>
using decay_t = typename decay<T>::type;

template<typename F, typename Tuple, size_t... Idx>
auto apply_helper(F&& f, Tuple&& tp, std::index_sequence<Idx...>)
    -> decltype(std::forward<F>(f)(std::get<Idx>(std::forward<Tuple>(tp))...)) {
  return std::forward<F>(f)(std::get<Idx>(std::forward<Tuple>(tp))...);
}

template<typename F, typename Tuple>
auto apply(F&& f, Tuple&& tp)
    -> decltype(apply_helper(std::forward<F>(f), std::forward<Tuple>(tp),
                             std::make_index_sequence<std::tuple_size<decay_t<Tuple>>::value>{})) {
  return apply_helper(std::forward<F>(f), std::forward<Tuple>(tp),
                      std::make_index_sequence<std::tuple_size<decay_t<Tuple>>::value>{});
}

template<typename F, typename... Args>
auto invoke(F&& f, Args&&... args) -> decltype(std::forward<F>(f)(std::forward<Args>(args)...)) {
  return std::forward<F>(f)(std::forward<Args>(args)...);
}

}  // namespace std

#endif
#endif  // REST_RPC_CPLUSPLUS_14_H_
#endif // CPP_14_PLUGINS_H


//------------------------
//end of cpp_14_plugins.h

namespace bz_robot
{
	namespace thread_rpc
	{
        class noncopyable
        {
		protected:
		    noncopyable() = default;
		    ~noncopyable() = default;
		private:
		    noncopyable(const noncopyable&) = delete;
		    const noncopyable& operator=( const noncopyable& ) = delete;
		};

		template <typename... Args, typename Func, std::size_t... Idx>
		void for_each(const std::tuple<Args...>& t, Func&& f, std::index_sequence<Idx...>) {
		    (void)std::initializer_list<int> { (f(std::get<Idx>(t)), void(), 0)...};
		}

		template <typename... Args, typename Func, std::size_t... Idx>
		void for_each_i(const std::tuple<Args...>& t, Func&& f, std::index_sequence<Idx...>) {
		    (void)std::initializer_list<int> { (f(std::get<Idx>(t), std::integral_constant<size_t, Idx>{}), void(), 0)...};
		}

		template<typename T>
		struct function_traits;

		//普通函数
		template<typename Ret, typename... Args>
		struct function_traits<Ret(Args...)>
		{
		public:
		    enum { arity = sizeof...(Args) };
		    typedef Ret function_type(Args...);
		    typedef Ret return_type;
		    using stl_function_type = std::function<function_type>;
		    typedef Ret(*pointer)(Args...);

		    typedef std::tuple<Args...> tuple_type;
		    typedef std::tuple<std::remove_const_t<std::remove_reference_t<Args>>...> bare_tuple_type;
		    using args_tuple = std::tuple<void*, std::remove_const_t<std::remove_reference_t<Args>>...>;
		    using args_tuple_2nd = std::tuple<void*, std::remove_const_t<std::remove_reference_t<Args>>...>;
		};

		template<typename Ret>
		struct function_traits<Ret()> {
		public:
		    enum { arity = 0 };
		    typedef Ret function_type();
		    typedef Ret return_type;
		    using stl_function_type = std::function<function_type>;
		    typedef Ret(*pointer)();

		    typedef std::tuple<> tuple_type;
		    typedef std::tuple<> bare_tuple_type;
		    using args_tuple = std::tuple<void*>;
		    using args_tuple_2nd = std::tuple<void*>;
		};

		template<typename Ret, typename... Args>
		struct function_traits<Ret(*)(Args...)> : function_traits<Ret(Args...)>{};

		template <typename Ret, typename... Args>
		struct function_traits<std::function<Ret(Args...)>> : function_traits<Ret(Args...)>{};

		template <typename ReturnType, typename ClassType, typename... Args>
		struct function_traits<ReturnType(ClassType::*)(Args...)> : function_traits<ReturnType(Args...)>{};

		template <typename ReturnType, typename ClassType, typename... Args>
		struct function_traits<ReturnType(ClassType::*)(Args...) const> : function_traits<ReturnType(Args...)>{};

		template<typename Callable>
		struct function_traits : function_traits<decltype(&Callable::operator())>{};

		template<typename T>
		using remove_const_reference_t = std::remove_const_t<std::remove_reference_t<T>>;

		template<size_t... Is>
		auto make_tuple_from_sequence(std::index_sequence<Is...>)->decltype(std::make_tuple(Is...)) {
		    std::make_tuple(Is...);
		}

		template<size_t N>
		constexpr auto make_tuple_from_sequence()->decltype(make_tuple_from_sequence(std::make_index_sequence<N>{})) {
		    return make_tuple_from_sequence(std::make_index_sequence<N>{});
		}

		namespace detail {
		    template <class Tuple, class F, std::size_t...Is>
		    void tuple_switch(const std::size_t i, Tuple&& t, F&& f, std::index_sequence<Is...>) {
		        (void)std::initializer_list<int> {
		                (i == Is && (
		                        (void)std::forward<F>(f)(std::integral_constant<size_t, Is>{}), 0))...
		        };
		    }
		} // namespace detail

		template <class Tuple, class F>
		inline void tuple_switch(const std::size_t i, Tuple&& t, F&& f) {
		    constexpr auto N =
		            std::tuple_size<std::remove_reference_t<Tuple>>::value;

		    detail::tuple_switch(i, std::forward<Tuple>(t), std::forward<F>(f),
		                         std::make_index_sequence<N>{});
		}

		template<int N, typename... Args>
		using nth_type_of = std::tuple_element_t<N, std::tuple<Args...>>;

		template<typename... Args>
		using last_type_of = nth_type_of<sizeof...(Args)-1, Args...>;



		//-------------
		// 遍历tuple
		//作者：菜鸡SHP
		//链接：https://www.zhihu.com/question/53397590/answer/1634928401
		//来源：知乎
		//著作权归作者所有。商业转载请联系作者获得授权，非商业转载请注明出处。



		template <size_t index = 0, typename... Args>
		typename std::enable_if<(index >= sizeof...(Args)), std::ostream&>::type
		operator<<(std::ostream& os, const std::tuple<Args...>& t)
		{
		    //int i_count = std::tuple_size<decltype(t)>::value;
            printf("for each1 tuple, sizeof...(Args) = %d\n", sizeof...(Args));
		    return os << (sizeof...(Args) == 0 ? "<>" : "");
		}

		template <size_t index = 0, typename... Args>
		typename std::enable_if<(index < sizeof...(Args)), std::ostream&>::type
		operator<<(std::ostream& os, const std::tuple<Args...>& t) {


		          //int i_count = std::tuple_size<decltype(t)>::value;
		          printf("for each2 tuple, sizeof...(Args) = %d\n", sizeof...(Args));
		    return operator<<<index + 1, Args...>(os
		                                              << (index == 0 ? "<" : "")
		                                              << std::get<index>(t)
		                                              << (index + 1 == sizeof...(Args) ? '>' : ','),
		                                          t);
		}


		//作者：Irons Du
		//链接：https://www.zhihu.com/question/53397590/answer/134847176
		//来源：知乎
		//著作权归作者所有。商业转载请联系作者获得授权，非商业转载请注明出处。

		//template<class Tuple, std::size_t N>
		//struct TupleReader
		//{
		//  static void read(Tuple& t)
		//  {
		//      TupleReader<Tuple, N - 1>::read(t);
		//      std::cout << std::get<N - 1>(t) << std::endl;
		//  }
		//};

		//template<class Tuple>
		//struct TupleReader < Tuple, 1 >
		//{
		//  static void read(Tuple& t)
		//  {
		//      std::cout << std::get<0>(t) << std::endl;
		//  }
		//};

		//-------------

		class ServiceData
		{
		public:
		    void init( const uint8_t& size)
		    {
		        mq.resize(size);
		        func_name_thread_index.clear();
		        //mutex_list.resize(size);
		        for(int i = 0; i < size; ++i)
		        {
		            p_mutex_list.emplace_back(new std::mutex());
		            p_service_condition_list.emplace_back(new std::condition_variable());
		            p_service_mutex_list.emplace_back(new std::mutex());
		        }
		        p_promise_list.resize(size);
		        p_return_result_list.resize(size);
                #ifdef __USE_SEM__
                    sem_handle_list.resize(size);
                #endif
		        //service_condition_mutex_list.resize(size);
		    }
		public:
		    std::vector<std::tuple<std::string, std::vector<void*>>* > mq;
		    std::unordered_map<std::string, uint8_t> func_name_thread_index;
		    std::vector<std::condition_variable*> p_service_condition_list;
		    std::vector<std::mutex*> p_service_mutex_list;
		    std::vector<std::mutex*> p_mutex_list;
		    std::vector<std::shared_ptr<std::promise<uint8_t> > > p_promise_list;
		    std::vector<void*> p_return_result_list;
            #ifdef __USE_SEM__
                std::vector<sem_t> sem_handle_list;
            #endif
		};


        class GlobalMQ : public noncopyable
        {
        public:
          static GlobalMQ& global_mq()
          {
            static GlobalMQ intance;
            return intance;
          }


          bool register_service(const std::string& name)
          {
              std::lock_guard<std::mutex> lock(m_mtx);
              auto it = m_mq.find(name);
              assert(it == m_mq.end());
              m_mq[name] = new ServiceData();
          }
          ServiceData* find_service_data(const std::string& name)
          {
              std::lock_guard<std::mutex> lock(m_mtx);
              auto it = m_mq.find(name);
              if(it == m_mq.end())
              {
                  PRINT_ERROR("can't find service: {}", name.c_str());
                  //exit(1);
                  return nullptr;
              }
              return (ServiceData*)(it->second);
          }
        private:
            std::unordered_map<std::string, ServiceData*> m_mq;
            std::mutex m_mtx;
        };


		class Client : public noncopyable
		{
		public:
		    Client(const std::string& service_name):
		        m_service_name(service_name),
		        m_is_connected(false)
		    {
		    }

		    bool connect()
		    {
		        if(!m_is_connected)
		        {
                    mp_service_data = GlobalMQ::global_mq().find_service_data(m_service_name);
		            m_is_connected = (mp_service_data != nullptr);
                    PRINT_INFO("connected {} finished, connect status = {}\n", m_service_name, m_is_connected);
		        }
		        return m_is_connected;
		    }

		    template<typename T=void, typename... Args>
		    typename std::enable_if<std::is_void<T>::value>::type  call(const std::string& name, Args&& ... args)
		    {
		        //RECORD_TIME();
                //PRINT_INFO("call {}\n", name.c_str());
		        connect();
                if(mp_service_data == nullptr)
                {
                    PRINT_ERROR("client can't connect to service: {}\n", m_service_name);
                    exit(-1);
                }
		        auto it = mp_service_data->func_name_thread_index.find(name);
		        if(it == mp_service_data->func_name_thread_index.end())
                {
                    PRINT_ERROR("server {} unsupported service: {}\n", m_service_name, name);
                    abort();
                }
		        //assert(it != mp_service_data->func_name_thread_index.end());
		        uint8_t index = it->second;
		        std::lock_guard<std::mutex> lock(*(mp_service_data->p_mutex_list[index]));
		        mp_service_data->p_promise_list[index] = std::make_shared<std::promise<uint8_t> >();
		        std::vector<void*> args_list;
		        int arr[] = {(class_push_arg(&args_list, &args), 0)...};
		        auto param = (std::make_tuple(name, args_list));
		        mp_service_data->mq[index] = &param;
		        mp_service_data->p_return_result_list[index] = nullptr;
                #if __USE_SEM__
                    if(sem_post( &(mp_service_data->sem_handle_list[index])) != 0 )
                    {
                        printf("error on pose sem, exit \n");
                        exit(-1);
                    }
                #else
                {
                    printf("notify server\n");
                    std::unique_lock<std::mutex> server_lock(*(mp_service_data->p_service_mutex_list[index]));
                    mp_service_data->p_service_condition_list[index]->notify_one();
                }
                #endif
                //std::this_thread::yield();
		        uint8_t err_code = mp_service_data->p_promise_list[index]->get_future().get();
		        if(err_code != 0)
		        {
                    PRINT_ERROR("call finished, error code = {}\n\n", err_code);
		            exit(err_code);
		        }
		        return;
            }

		    template<typename T, typename... Args>
		    typename std::enable_if<!std::is_void<T>::value, T>::type call(const std::string& name, Args&& ... args)
		    {
		        //RECORD_TIME();
                //PRINT_INFO("call {}\n", name.c_str());
		        connect();
                //assert(mp_service_data != nullptr);
                if(mp_service_data == nullptr)
                {
                    PRINT_ERROR("client can't connect to service: {}\n", m_service_name);
                    exit(-1);
                }
		        auto it = mp_service_data->func_name_thread_index.find(name);
                if(it == mp_service_data->func_name_thread_index.end())
                {
                    PRINT_ERROR("server {} unsupported service: {}\n", m_service_name, name);
                    abort();
                }
                //assert(it != mp_service_data->func_name_thread_index.end());
		        uint8_t index = it->second;
		        std::lock_guard<std::mutex> lock(*(mp_service_data->p_mutex_list[index]));
		        mp_service_data->p_promise_list[index] = std::make_shared<std::promise<uint8_t> >();
		        std::vector<void*> args_list;
		        int arr[] = {(class_push_arg(&args_list, &args), 0)...};
		        auto param = (std::make_tuple(name, args_list));
		        mp_service_data->mq[index] = &param;
		        T result;
		        //std::shared_ptr<T> result = std::make_shared<T>();
		        mp_service_data->p_return_result_list[index] = (void*)(&result);
                #if __USE_SEM__
                    if(sem_post( &(mp_service_data->sem_handle_list[index])) != 0 )
                    {
                        printf("error on pose sem, exit \n");
                        exit(-1);
                    }
                #else
                {
                    printf("notify server\n");
                    std::unique_lock<std::mutex> server_lock(*(mp_service_data->p_service_mutex_list[index]));
                    mp_service_data->p_service_condition_list[index]->notify_one();
                }
                #endif
                //std::this_thread::yield();
		        uint8_t err_code = mp_service_data->p_promise_list[index]->get_future().get();
		        if(err_code != 0)
		        {
                    PRINT_ERROR("call finished, error code = {}\n\n", err_code);
		            exit(err_code);
		        }
		        //std::cout << "call result " << *reinterpret_cast<T*>(mp_service_data->p_return_result_list[index]) << std::endl;
		        //T result = *reinterpret_cast<T*>(mp_service_data->p_return_result_list[index]);
		        //std::cout << "result = " << result << std::endl;
		        //return  result;

        		return result;
		    };

		private:
		    template <class T>
		    void class_push_arg(std::vector<void*> *p_args_list, T* t)
		    {
		       //cout << t << endl;
		#if 0
		       T* p = new T;
		       *p = t;
		       p_args_list->emplace_back((void*) p);
		#else
		       p_args_list->emplace_back((void*)(reinterpret_cast<T*>(t)));
		#endif

		    }
		private:
		    const std::string m_service_name;
		    bool m_is_connected;
		    ServiceData *mp_service_data;
		};



		///
		template <size_t index = 0, typename... Args>
		typename std::enable_if<(index >= sizeof...(Args))>::type
		for_each_tuple_pack(std::tuple<Args...> &t, const std::vector<void*>& p_args)
		{
		  //printf("for each1 tuple, sizeof...(Args) = %d\n", sizeof...(Args));
		  return;
		}

		template <size_t index = 0, typename... Args>
		typename std::enable_if<(index < sizeof...(Args))>::type
		for_each_tuple_pack(std::tuple<Args...> &t, const std::vector<void*>& p_args)
		{
		    //int i_count = std::tuple_size<decltype(t)>::value;
		    //printf("for each2 tuple, sizeof...(Args) = %d, index = %d\n", sizeof...(Args), index);
		    //std::cout <<  std::get<index>(t) << std::endl;
		    //std::cout << "typeid(t).name() : " << typeid(t).name() << std::endl;
		    auto t_m = std::get<index>(t);
		    //std::cout << "tm type: " << typeid(t_m).name() << std::endl;
            if(index > 0)
		    {
		        //printf("args: %p\n", p_args[index-1]);
                if(p_args.size() > index -1 )
                {
                    auto* p_tm = reinterpret_cast<decltype(t_m)*>(p_args[index-1]);
                    //std::cout << *p_tm << std::endl;
                    t_m = *p_tm;
                    std::get<index>(t) = *p_tm;
                }


		    }
		    for_each_tuple_pack<index + 1, Args...>(t, p_args);
		}
		///
		///


        template<typename T>
        T unpack(std::vector<void*> p_args)
        {
            T tp;
            for_each_tuple_pack(tp, p_args);
            return tp;
#if 0
            try
            {
                T tp;
                for_each_tuple_pack(tp, p_args);
                return tp;
            }
            catch(std::exception& e)
            {
                PRINT_ERROR("unpack exception: {}", e.what());
            }
            catch(...)
            {
                PRINT_ERROR("unpack unknown error\n");
            }
            return T();
#endif
        }



		class Server : public noncopyable
		{
		private:
            class Router
            {
            public:
                template<typename Function>
                void register_handler(std::string const& name, Function f)
                {
                    return register_nonmember_func(name, std::move(f));
                }

                template<typename Function, typename Self>
                void register_handler(std::string const& name, const Function& f, Self* self) {
                    return register_member_func(name, f, self);
                }

                //void remove_handler(std::string const& name) { this->map_invokers_.erase(name); }

                //Router() = default;

                const std::unordered_map<std::string, std::function<void(std::vector<void*>, void *)>> &
                    map_invokers()
                {
                    return map_invokers_;
                }
            private:

                template<typename F, size_t... I, typename Arg, typename... Args>
                static typename std::result_of<F(Args...)>::type
                call_helper(const F & f, const std::index_sequence<I...>&, std::tuple<Arg, Args...> tup)
                {
                    return f(std::move(std::get<I + 1>(tup))...);
                }

                template<typename F, typename Arg, typename... Args>
                static
                    typename std::enable_if<std::is_void<typename std::result_of<F(Args...)>::type>::value>::type
                    call(const F & f, void *p_result, std::tuple<Arg, Args...> tp)
                {
                    call_helper(f, std::make_index_sequence<sizeof...(Args)>{}, std::move(tp));
                    //*p_result = nullptr;
                }

                template<typename F, typename Arg, typename... Args>
                static
                    typename std::enable_if<!std::is_void<typename std::result_of<F(Args...)>::type>::value>::type
                    call(const F & f, void *p_result, std::tuple<Arg, Args...> tp)
                {
                    auto r = call_helper(f, std::make_index_sequence<sizeof...(Args)>{}, std::move(tp));
                    //std::cout << "result = " << r << std::endl;
                    //static auto result(r);
                    //result = r;
			        if(p_result)
			        {
			            *(reinterpret_cast<decltype(r)*>(p_result)) = r;
			        }
			    }

                template<typename F, typename Self, size_t... Indexes, typename Arg, typename... Args>
                static typename std::result_of<F(Self, Args...)>::type
                call_member_helper(const F & f, Self * self, const std::index_sequence<Indexes...>&, std::tuple<Arg, Args...> tup)
                {
                    return (*self.*f)(std::move(std::get<Indexes + 1>(tup))...);
                }

                template<typename F, typename Self, typename Arg, typename... Args>
                static typename std::enable_if<
                    std::is_void<typename std::result_of<F(Self, Args...)>::type>::value>::type
                    call_member(const F & f, Self * self, void *p_result,std::tuple<Arg, Args...> tp)
                {
                    call_member_helper(f, self, typename std::make_index_sequence<sizeof...(Args)>{}, std::move(tp));
                    //*p_result = nullptr;
                }

                template<typename F, typename Self, typename Arg, typename... Args>
                static typename std::enable_if<!std::is_void<typename std::result_of<F(Self, Args...)>::type>::value>::type
                    call_member(const F & f, Self * self, void *p_result,std::tuple<Arg, Args...> tp) {
                    auto r =
                        call_member_helper(f, self, typename std::make_index_sequence<sizeof...(Args)>{}, std::move(tp));
			        if(p_result)
			        {
			            *(reinterpret_cast<decltype(r)*>(p_result)) = r;
			        }
                }

                template<typename Function>
                struct invoker
                {
                    //template<typename T>
                    static inline void apply_func(const Function& func, std::vector<void*> p_args, void *p_result)
                    {
                        //printf("apply_func\n");
                        using args_tuple = typename function_traits<Function>::args_tuple_2nd;
                        auto tp = unpack<args_tuple>(p_args);
                        call(func, p_result, std::move(tp));

                    }

                    template<typename Self>
                    static inline void apply_member_func(const Function& func, Self* self, std::vector<void*> p_args,
                                                         void *p_result)
                    {
                        using args_tuple = typename function_traits<Function>::args_tuple_2nd;
                        auto tp = unpack<args_tuple>(p_args);
                        call_member(func, self, p_result, std::move(tp));
                    }
                };

                template<typename Function>
                void register_nonmember_func(const std::string& name, Function f)
                {
                    this->map_invokers_[name] = {std::bind(&invoker<Function>::template apply_func, std::move(f), std::placeholders::_1,
                                                           std::placeholders::_2)};
                }

                template<typename Function, typename Self>
                void register_member_func(const std::string& name, const Function& f, Self* self)
                {
                    this->map_invokers_[name] = {std::bind(&invoker<Function>::template apply_member_func<Self>,
                                                           f, self, std::placeholders::_1, std::placeholders::_2)};
                }

                std::unordered_map<std::string,
                    std::function<void(std::vector<void*>, void *)>>
                    map_invokers_;
            }; //end of class Router
		public:
		    Server(const std::string& service_name, uint8_t thread_pool_size = 1):
		        m_service_name(service_name)
		    {
                GlobalMQ::global_mq().register_service(service_name);
                mp_service_data = GlobalMQ::global_mq().find_service_data(service_name);
		        thread_pool_size = std::max(thread_pool_size, (uint8_t)1);
		        mp_service_data->init(thread_pool_size);
		        mp_thread_pool.resize(thread_pool_size);
		        m_router_list.resize(thread_pool_size);
                #ifdef __USE_SEM__
                for(int i = 0; i < thread_pool_size; ++i)
                {
                    if(sem_init(&(mp_service_data->sem_handle_list[i]), 0, 0) == -1)    //初始化信号量
                    {
                        printf("error: sem_init failed!\r\n");
                        exit(0);
                    }
                }
                #endif
                PRINT_INFO("----------\tinit server: {}\t----------\n", service_name.c_str());
                //std::cout << "init server:  " << service_name << std::endl;
		    }


		    template<typename Function>
		    void register_handler(std::string const& name, const Function& f, const uint8_t thread_index = 0)
		    {
		        //printf("register_handler1\n");
                        ServiceData *p_service_data = GlobalMQ::global_mq().find_service_data(m_service_name);
		        assert(p_service_data != nullptr);
		        std::lock_guard<std::mutex> lock(*(p_service_data->p_mutex_list[thread_index]));
		        p_service_data->func_name_thread_index[name] = thread_index;
		        m_router_list[thread_index].register_handler(name, f);

		    }

		    template<typename Function, typename Self>
		    void register_handler(std::string const& name, const Function& f, Self* self, const uint8_t thread_index = 0)
		    {
		        //printf("register_handler2\n");
                        ServiceData *p_service_data = GlobalMQ::global_mq().find_service_data(m_service_name);
		        assert(p_service_data != nullptr);
		        std::lock_guard<std::mutex> lock(*(p_service_data->p_mutex_list[thread_index]));
		        p_service_data->func_name_thread_index[name] = thread_index;
		        m_router_list[thread_index].register_handler(name, f, self);
		    }


		    void run()
		    {
                printf("server: \t%s\trunning ...\n", m_service_name.c_str());
		        for(uint8_t i = 0; i < mp_thread_pool.size(); ++i)
		        {
		            //printf("before create thread %d\n", i);
                    mp_thread_pool[i] = new std::thread(&Server::work_thread, this, i);
		            //printf("after create thread %d\n", i);
		        }
		        //printf("2333\n");
                for(uint8_t i = 0; i < mp_thread_pool.size(); ++i)
		        {
		            //printf("%d thread detach\n", i);
                    mp_thread_pool[i]->join();
		            //mp_thread_pool[i]->join();
		        }
		        //printf("3222\n");
                //mp_thread_pool[0]->join();
                PRINT_ERROR("rpc server {} exit", m_service_name);
                exit(-1);
		    }
		private:
            void work_thread(const uint8_t thread_index)
		    {
                PRINT_INFO("server {} work thread: {}\n", m_service_name.c_str(), thread_index);
		        //wait client set task
		        //mp_service_data->p_service_mutex_list[thread_index]->lock();
                LOOP:
                std::shared_ptr<std::promise<uint8_t> > p_promise;
                p_promise.reset();
                try
                {
		        while (true)
		        {
		            //printf("server  prepare acquire server lock\n");
                    #if __USE_SEM__
                        if(sem_wait( &(mp_service_data->sem_handle_list[thread_index])) != 0 )
                        {
                            printf("error on wait sem, exit \n");
                            exit(-1);
                        }
                    #else
                        std::unique_lock<std::mutex> server_lock(*(mp_service_data->p_service_mutex_list[thread_index]));
                        mp_service_data->p_service_condition_list[thread_index]->wait(server_lock);
                    #endif

                    p_promise = mp_service_data->p_promise_list[thread_index];
		            //printf("server  recv client notify\n");
		            //RECORD_TIME();
		            //防止虚假唤醒
		            if(mp_service_data->mq[thread_index] != nullptr)
		            {
		                const std::string& func_name = std::get<0>(*(mp_service_data->mq[thread_index]));
		                //std::cout << "server  function name = " << func_name << std::endl;
                        //PRINT_DEBUG("running function : {}", func_name);
		                auto it = m_router_list[thread_index].map_invokers().find(func_name);
		                if(it != m_router_list[thread_index].map_invokers().end())
		                {
                            //printf("server  prepare call function\n");
		                    //std::string result;
		                    //mp_service_data->p_return_result_list[thread_index] = nullptr;
		                    it->second(std::get<1>(*(mp_service_data->mq[thread_index])),
		                               mp_service_data->p_return_result_list[thread_index]);
		                    //std::cout << "server result addr " << mp_service_data->p_return_result_list[thread_index] << std::endl;
		                    mp_service_data->p_promise_list[thread_index]->set_value(0);
                            //PRINT_DEBUG("function : {} run finished", func_name);
		                }
		                else
		                {
                            PRINT_ERROR("server{} error, unsuppored func name: %s\n", m_service_name, func_name.c_str());
		                    mp_service_data->p_promise_list[thread_index]->set_value(1);
		                }
		            }
		            else
		            {
		                printf("server recv fake notify\n");
		            }
		        }
                }
                catch(std::exception& e)
                {
                    PRINT_ERROR("wotk thread: {} exception: {}", m_service_name, e.what());
                    p_promise->set_value(1);
                    //std::cout << "wotk thread: " << m_service_name << " exception " << e.what() << std::endl;
                    goto LOOP;
                }
                catch(...)
                {
                    PRINT_ERROR("unpack unknown error\n");
                    p_promise->set_value(1);
                    //std::cout << "unpack unknown error\n" << std::endl;
                    goto LOOP;
                }
		    }

		private:
		    const std::string m_service_name;
		    bool m_is_connected;
		    ServiceData *mp_service_data;
		    //Router m_router;
		    std::vector<Router> m_router_list;
		    std::vector<std::thread*> mp_thread_pool;
		};

		
		// examples
		#if 0

		//test
		void hello()
		{
		    printf("hello\n");
		}

		//test
		double version()
		{
		    return 3.14;
		}

		//test
		void check(double ver)
		{
		    printf("%f\n", ver);
		}

		//test
		int double_to_int(double ver)
		{
		    return int(ver);
		}


		int add(int a, int b)
		{
		    printf("add: a = %d\n", a);
		    printf("add: b = %d\n", b);
		    return a+b;
		}

		int add3(int a, int b, int c)
		{
		    printf("add3: a = %d\n", a);
		    printf("add3: b = %d\n", b);
		    printf("add3: c = %d\n", c);
		    return a+b+c;
		}

		int check_sequence(int a, std::string b, double c)
		{
		    std::cout << "check_sequence  " << a << std::endl;
		    std::cout << "check_sequence  " << b << std::endl;
		    std::cout << "check_sequence  " << c << std::endl;
		    return a + b.size() +c;
		}

		class A
		{
		public:
		    double add(const double &a, double b)
		    {
		        std::cout << "a_add  " << a << std::endl;
		        std::cout << "a_add  " << b << std::endl;
		        std::cout << "a_add a+b =   " << a+b << std::endl;
		        return a+b;
		    }
		};

		void module_server1()
		{
		    A a;
		    Server server1("test-server1", 2);
		    server1.register_handler("hello", hello);
		    server1.register_handler("version", version);
		    server1.register_handler("check", check);
		    server1.register_handler("add", add);
		    server1.register_handler("add3", add3, 1);
		    server1.register_handler("check_sequence", check_sequence);
		    server1.register_handler("a_add", &A::add, &a);
		    server1.run();
		}



		void thread_client_tester1()
		{
		    std::this_thread::sleep_for(std::chrono::seconds(2));
		    printf("init client\n");
		    Client client1("test-server1");

		    while (true)
		    {
		//        int result = client1.call<int>("add3", 2, 3, 3);
		//        printf("thread_client_tester1 call add3 result = %d\n", result);
		        std::this_thread::sleep_for(std::chrono::milliseconds(500));

		        double result2 = client1.call<double>("a_add", 3.54, (double)3);
		        printf("thread_client_tester1 call a_add result = %f\n", result2);
		    }
		}
		
		int main()
		{
		    std::thread server1_thread = std::thread(module_server1);
		    server1_thread.detach();
		//    std::thread client_tester1_thread = std::thread(thread_client_tester1);
		//    client_tester1_thread.detach();

		    std::this_thread::sleep_for(std::chrono::seconds(2));
		    printf("init client\n");
		    Client client1("test-server1");
		    while (true)
		    {
		//        printf("time stamp: [%.3f]\n", time_stamp());
		        int a = 1;
		        std::string b = "abcdcba";
		        double c = 3.14;
		        int result = client1.call<int>("check_sequence", a, std::string("123"), 12.222);
		        printf("main result = %d\n", result);
		        std::this_thread::sleep_for(std::chrono::milliseconds(500));
		    }
		    printf("exit 0");
		    return 0;
		}
		#endif


	}
}
#endif //__THREAD_RPC__
