#include "bitbot_kernel/kernel/kernel_data.hpp"
#include "bitbot_kernel/utils/logger.h"
#include "bitbot_kernel/types.hpp"

#include "glaze/glaze.hpp"
#include "readerwriterqueue.h"

#include <fstream>
#include <thread>
#include <map>
#include <unordered_map>
#include <atomic>
#include <optional>

namespace bitbot
{
  class Backend
  {
  public:
    Backend(int listen_port = 12888);
    ~Backend();

    void SetPort(int port);
    void RegisterSettingsFile(std::string file);

    void SetMonitorHeaders(std::string json_str);
    void SetMonitorData(const std::vector<Number>& data);

    void SetStatesList(std::vector<std::pair<StateId, std::string>> states);
    void SetEventsMap(std::unordered_map<std::string, EventId> *events_map);

    inline bool GetEvent(std::vector<std::pair<EventId, EventValue>> &event)
    {
      return events_queue_.try_dequeue(event);
    }

    void Run();

    void Stop();

  private:
    void Running();
    
    void UpdateData();

    struct PerSocketData {
        /* Fill with user data */
    };

    struct WebsocketMessageType
    {
      std::string type; // message type "request_data" "events" "monitor_data"
      std::string data; // message data

      struct glaze {
        using T = WebsocketMessageType;
        static constexpr auto value = glz::object(
          "type", &T::type,
          "data", &T::data
        );
      };
    };

    struct MonitorData
    {
      std::vector<Number> data;
      
      struct glaze {
        using T = MonitorData;
        static constexpr auto value = glz::object(
          "data", &T::data
        );
      };
    };

    struct BackendControlSetting
    {
      std::string event;
      std::string kb_key;

      struct glaze {
        using T = BackendControlSetting;
        static constexpr auto value = glz::object(
          "event", &T::event,
          "kb_key", &T::kb_key
        );
      };
    };

    struct BackendSettings
    {
      std::vector<BackendControlSetting> control;
      
      struct glaze {
        using T = BackendSettings;
        static constexpr auto value = glz::object(
          "control", &T::control
        );
      };
    };

    struct EventType
    {
      std::string name;
      EventValue value;
      
      struct glaze {
        using T = EventType;
        static constexpr auto value = glz::object(
            "name", &T::name,
            "value", &T::value
        );
      };
    };

    struct EventsType
    {
      std::vector<EventType> events;
      
      struct glaze {
        using T = EventsType;
        static constexpr auto value = glz::object(
          "events", &T::events
        );
      };
    };

    struct StateType
    {
      StateId id;
      std::string name;

      struct glaze {
        using T = StateType;
        static constexpr auto value = glz::object(
          "id", &T::id,
          "name", &T::name
        );
      };
    };

    struct StatesType
    {
      std::vector<StateType> states;

      struct glaze {
        using T = StatesType;
        static constexpr auto value = glz::object(
          "states", &T::states
        );
      };
    };

    Logger::Console logger_;

    bool run_ = false;
    int listen_port_;
    BackendSettings settings_;

    std::string monitor_headers_json_;
    std::atomic<bool> is_data_update_ = false;
    std::atomic<bool> is_reading_data_ = false;
    MonitorData monitor_data_;
    std::string monitor_data_str_;

    StatesType states_list_;
    std::string states_list_str_;

    std::unordered_map<std::string, EventId> *events_name_id_map_ = nullptr;

    moodycamel::ReaderWriterQueue<std::vector<std::pair<EventId, EventValue>>> events_queue_;
    
    std::unique_ptr<std::thread> running_thread_ = nullptr;
  };

}


