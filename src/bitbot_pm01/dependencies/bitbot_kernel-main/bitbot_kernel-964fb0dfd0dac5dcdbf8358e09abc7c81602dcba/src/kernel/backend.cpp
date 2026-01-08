#include "bitbot_kernel/kernel/backend.h"
#include "bitbot_kernel/utils/time_func.h"
#include "bitbot_kernel/utils/assert.h"

#include "App.h"

namespace bitbot
{
  namespace {
    uWS::App* uws_app_ptr = nullptr;
    uWS::Loop* uws_loop = nullptr;
  }

  Backend::Backend(int listen_port)
  : logger_(Logger().ConsoleLogger())
  , listen_port_(listen_port)
  , events_queue_(16)
  , running_thread_(nullptr)
  {

  }

  Backend::~Backend()
  {
    Stop();
  }

  void Backend::SetPort(int port)
  {
    listen_port_ = port;
  }

  void Backend::RegisterSettingsFile(std::string file)
  {
    auto buffer = glz::file_to_buffer(file);
    auto ec = glz::read<glz::opts{.error_on_unknown_keys = false}>(settings_, buffer);
    if (ec) {
      logger_->error(glz::format_error(ec, buffer));
    }
  }

  void Backend::SetMonitorHeaders(std::string str)
  {
    monitor_headers_json_ = str;
  }

  void Backend::SetStatesList(std::vector<std::pair<StateId, std::string>> states)
  {
    for(auto& state:states)
    {
      states_list_.states.push_back({.id = state.first, .name = state.second});
    }
    states_list_str_ = glz::write_json(states_list_);
  }

  void Backend::SetEventsMap(std::unordered_map<std::string, EventId> *events_map)
  {
    events_name_id_map_ = events_map;
  }

  void Backend::Run()
  {
    if(!run_)
    {
      bitbot_assert(events_name_id_map_, "Backend events_name_id_map_ must have valid value");
      if(running_thread_ == nullptr)
      {
        running_thread_ = std::make_unique<std::thread>(&Backend::Running, this);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
      }
      run_ = true;
    }
  }

  void Backend::Stop()
  {
    if(uws_app_ptr && run_)
    {
      uws_loop->defer([this](){
        uws_app_ptr->close();
        uws_app_ptr = nullptr;
      });
      if(running_thread_ && running_thread_->joinable())
      {
        running_thread_->join();
        running_thread_ = nullptr;
      }
      run_ = false;
    }
  }

  void Backend::SetMonitorData(const std::vector<Number>& data)
  {
    if(!is_reading_data_.load())
    {
      is_reading_data_.store(true);
      monitor_data_.data = data;
      is_reading_data_.store(false);
      is_data_update_.store(true);
    }
  }

  void Backend::Running()
  {
    uWS::App uws_app;
    uws_app_ptr = &uws_app;
    // uws_app = std::make_unique<uWS::App>();
    uws_app.get("/monitor/headers", [this](auto *res, auto */*req*/) {
      // std::cout << "http req from " << res->getRemoteAddressAsText() << std::endl;
      res->writeHeader("Access-Control-Allow-Origin", "*");
      res->writeHeader("Access-Control-Allow-Methods", "GET, POST, PUT, DELETE, OPTIONS");
      res->writeHeader("Access-Control-Allow-Headers", "origin, content-type, accept, x-requested-with");
      res->writeHeader("Access-Control-Max-Age", "3600");
      res->end(this->monitor_headers_json_);
    }).get("/monitor/stateslist", [this](auto *res, auto */*req*/) {
      // std::cout << "http req from " << res->getRemoteAddressAsText() << std::endl;
      res->writeHeader("Access-Control-Allow-Origin", "*");
      res->writeHeader("Access-Control-Allow-Methods", "GET, POST, PUT, DELETE, OPTIONS");
      res->writeHeader("Access-Control-Allow-Headers", "origin, content-type, accept, x-requested-with");
      res->writeHeader("Access-Control-Max-Age", "3600");
      res->end(this->states_list_str_);
    }).get("/setting/control/get", [this](auto *res, auto */*req*/) {
      // std::cout << "http setting/get req from " << res->getRemoteAddressAsText() << std::endl;
      res->writeHeader("Access-Control-Allow-Origin", "*");
      res->writeHeader("Access-Control-Allow-Methods", "GET, POST, PUT, DELETE, OPTIONS");
      res->writeHeader("Access-Control-Allow-Headers", "origin, content-type, accept, x-requested-with");
      res->writeHeader("Access-Control-Max-Age", "3600");
      res->end(glz::write_json(settings_.control));
    }).ws<PerSocketData>("/console", {
      /* Settings */
      .compression = uWS::CompressOptions(uWS::DEDICATED_COMPRESSOR_4KB | uWS::DEDICATED_DECOMPRESSOR),
      .maxPayloadLength = 100 * 1024 * 1024,
      .idleTimeout = 16,
      .maxBackpressure = 100 * 1024 * 1024,
      .closeOnBackpressureLimit = false,
      .resetIdleTimeoutOnSend = false,
      .sendPingsAutomatically = true,
      /* Handlers */
      .upgrade = nullptr,
      .open = [this](auto */*ws*/) {
          /* Open event here, you may access ws->getUserData() which points to a PerSocketData struct */

        // std::cout << "A data WebSocket connected! " << ws->getRemoteAddressAsText() << std::endl;
      },
      .message = [this](auto *ws, std::string_view message, uWS::OpCode /*opCode*/) {
        static WebsocketMessageType msg_send;
        static WebsocketMessageType msg_receive;
        // logger_->debug("console message: {}", message);

        auto ec = glz::read<glz::opts{.error_on_unknown_keys = false}>(msg_receive, message);

        if(ec)
        {
          logger_->error(glz::format_error(ec, message));
        }
        else
        {
          if(msg_receive.type.compare("request_data") == 0)
          {
            UpdateData();
            msg_send.type = "monitor_data";
            msg_send.data = monitor_data_str_;
            ws->send(glz::write_json(msg_send), uWS::TEXT, true);
          }
          else if(msg_receive.type.compare("events") == 0)
          {
            EventsType events;
            ec = glz::read<glz::opts{.error_on_unknown_keys = false}>(events, msg_receive.data);
        
            if(ec)
            {
              logger_->error(glz::format_error(ec, msg_receive.data));
            }
            else
            {
              if(!events.events.empty())
              {
                std::vector<std::pair<std::string,EventValue>> cmds;
                std::vector<std::pair<EventId, EventValue>> temp_events;
                for(auto& event:events.events)
                {
                  cmds.push_back(std::pair<std::string,int>(event.name,event.value));
                  if(auto i=events_name_id_map_->find(event.name); i!=events_name_id_map_->end())
                  {
                    temp_events.push_back(std::pair<EventId, EventValue>(i->second, event.value));
                  }
                }
                events_queue_.enqueue(temp_events);
              }
            }
          }
        }
      },
      .drain = [](auto */*ws*/) {
          /* Check ws->getBufferedAmount() here */
      },
      .ping = [](auto */*ws*/, std::string_view) {
          /* Not implemented yet */
      },
      .pong = [](auto */*ws*/, std::string_view) {
          /* Not implemented yet */
      },
      .close = [](auto */*ws*/, int /*code*/, std::string_view /*message*/) {
          /* You may access ws->getUserData() here */

        // std::cout << "ws console Connection closed" << std::endl;
      }
    }).listen(listen_port_, [this](auto *socket) {
      if (socket)
      {
        logger_->info("Backend is listening on port {}", this->listen_port_);
        uws_loop = uWS::Loop::get();
      }
      else
      {
        logger_->error("Backend failed to listen on port {}", this->listen_port_);
      }
    }).run();
  }

  void Backend::UpdateData()
  {
    if(is_data_update_.load())
    {
      is_reading_data_.store(true);
      monitor_data_str_ = glz::write_json(monitor_data_);
      is_reading_data_.store(false);
      is_data_update_.store(false);
    }
  }  
}
