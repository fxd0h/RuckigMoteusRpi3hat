// Copyright 2020 Josh Pieper, jjp@pobox.com.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/// @file
///
/// This is a simple application that demonstrates how to efficiently
/// monitor and control multiple moteus servos at a high rate using
/// the pi3hat.
///
/// It is contained in a single file for the purposes of
/// demonstration.  A real application should likely be implemented in
/// multiple translation units or structured for longer term
/// maintenance.

#include <sys/mman.h>

#include <chrono>
#include <iomanip>
#include <iostream>
#include <future>
#include <limits>
#include <map>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "mjbots/moteus/moteus_protocol.h"
#include "mjbots/moteus/pi3hat_moteus_interface.h"

#include "joystick/joystick.hh"

#include <ruckig/ruckig.hpp>

using namespace mjbots;

using MoteusInterface = moteus::Pi3HatMoteusInterface;

namespace
{
  struct Arguments
  {
    Arguments(const std::vector<std::string> &args)
    {
      for (size_t i = 0; i < args.size(); i++)
      {
        const auto &arg = args[i];
        if (arg == "-h" || arg == "--help")
        {
          help = true;
        }
        else if (arg == "--main-cpu")
        {
          main_cpu = std::stoull(args.at(++i));
        }
        else if (arg == "--can-cpu")
        {
          can_cpu = std::stoull(args.at(++i));
        }
        else if (arg == "--period-s")
        {
          period_s = std::stod(args.at(++i));
        }
        else if (arg == "--primary-id")
        {
          primary_id = std::stoull(args.at(++i));
        }
        else if (arg == "--primary-bus")
        {
          primary_bus = std::stoull(args.at(++i));
        }
        else if (arg == "--secondary-id")
        {
          secondary_id = std::stoull(args.at(++i));
        }
        else if (arg == "--secondary-bus")
        {
          secondary_bus = std::stoull(args.at(++i));
        }
        else if (arg == "--joy-speed")
        {
          joy_speed = std::stod(args.at(++i));
        }
        else
        {
          throw std::runtime_error("Unknown argument: " + arg);
        }
      }
    }

    bool help = false;
    int main_cpu = 1;
    int can_cpu = 2;
    double period_s = 0.005;
    int primary_id = 1;
    int primary_bus = 3;
    int secondary_id = 2;
    int secondary_bus = 2;
    double joy_speed = 1;
  };

  void DisplayUsage()
  {
    std::cout << "Usage: moteus_control_example [options]\n";
    std::cout << "\n";
    std::cout << "  -h, --help           display this usage message\n";
    std::cout << "  --main-cpu CPU       run main thread on a fixed CPU [default: 1]\n";
    std::cout << "  --can-cpu CPU        run CAN thread on a fixed CPU [default: 2]\n";
    std::cout << "  --period-s S         period to run control\n";
    std::cout << "  --primary-id ID      servo ID of primary, undriven servo\n";
    std::cout << "  --primary-bus BUS    bus of primary servo\n";
    std::cout << "  --secondary-id ID    servo ID of secondary, driven servo\n";
    std::cout << "  --secondary-bus BUS  bus of secondary servo\n";
    std::cout << "  --joy-speed SPD      joystick absolute maximun speed\n";
  }

  void LockMemory()
  {
    // We lock all memory so that we don't end up having to page in
    // something later which can take time.
    {
      const int r = ::mlockall(MCL_CURRENT | MCL_FUTURE);
      if (r < 0)
      {
        throw std::runtime_error("Error locking memory");
      }
    }
  }

  std::pair<double, double> MinMaxVoltage(
      const std::vector<MoteusInterface::ServoReply> &r)
  {
    double rmin = std::numeric_limits<double>::infinity();
    double rmax = -std::numeric_limits<double>::infinity();

    for (const auto &i : r)
    {
      if (i.result.voltage > rmax)
      {
        rmax = i.result.voltage;
      }
      if (i.result.voltage < rmin)
      {
        rmin = i.result.voltage;
      }
    }

    return std::make_pair(rmin, rmax);
  }

  /// This holds the user-defined control logic.
  class SampleController
  {
  public:
    SampleController(const Arguments &arguments) : arguments_(arguments)
    {
      if ((arguments_.primary_bus == arguments_.secondary_bus) &&
          (arguments_.primary_id == arguments_.secondary_id))
      {
        throw std::runtime_error("Servos on the same bus must have unique IDs");
      }
    }

    /// This is called before any control begins, and must return the
    /// set of servos that are used, along with which bus each is
    /// attached to.
    std::vector<std::pair<int, int>> servo_bus_map() const
    {
      return {
          {arguments_.primary_id, arguments_.primary_bus},
          {arguments_.secondary_id, arguments_.secondary_bus},
      };
    }

    /// This is also called before any control begins.  @p commands will
    /// be pre-populated with an entry for each servo returned by
    /// 'servo_bus_map'.  It can be used to perform one-time
    /// initialization like setting the resolution of commands and
    /// queries.
    void Initialize(std::vector<MoteusInterface::ServoCommand> *commands)
    {
      moteus::PositionResolution res;
      moteus::QueryCommand query;

      res.position = moteus::Resolution::kFloat;
      res.velocity = moteus::Resolution::kFloat;

      // res.position = moteus::Resolution::kInt16;
      // res.velocity = moteus::Resolution::kInt16;
      res.feedforward_torque = moteus::Resolution::kInt16;
      res.kp_scale = moteus::Resolution::kInt16;
      res.kd_scale = moteus::Resolution::kInt16;
      res.maximum_torque = moteus::Resolution::kIgnore;
      res.stop_position = moteus::Resolution::kIgnore;
      res.watchdog_timeout = moteus::Resolution::kIgnore;

      query.position = moteus::Resolution::kFloat;
      query.mode = moteus::Resolution::kInt16;
      // query.position = moteus::Resolution::kInt16;
      query.velocity = moteus::Resolution::kFloat;
      query.torque = moteus::Resolution::kInt16;
      query.q_current = moteus::Resolution::kIgnore;
      query.d_current = moteus::Resolution::kIgnore;
      query.rezero_state = moteus::Resolution::kIgnore;
      query.voltage = moteus::Resolution::kInt8;
      query.temperature = moteus::Resolution::kInt8;
      query.fault = moteus::Resolution::kInt8;

      for (auto &cmd : *commands)
      {
        cmd.resolution = res;
        cmd.query = query;
      }
    }

    moteus::QueryResult Get(const std::vector<MoteusInterface::ServoReply> &replies, int id, int bus)
    {
      for (const auto &item : replies)
      {
        if (item.id == id && item.bus == bus)
        {
          return item.result;
        }
      }
      return {};
    }

    /// This is run at each control cycle.  @p status is the most recent
    /// status of all servos (note that it is possible for a given
    /// servo's result to be omitted on some frames).
    ///
    /// @p output should hold the desired output.  It will be
    /// pre-populated with the result of the last command cycle, (or
    /// Initialize to begin with).
    void RunWithAttributes(const std::vector<MoteusInterface::ServoReply> &status,
                           std::vector<MoteusInterface::ServoCommand> *output,
                           float reqPos, float reqSpeed)
    {
      cycle_count_++;

      // This is where your control loop would go.
      auto &primary_out = output->at(0);

      if (cycle_count_ < 5)
      {
        primary_out.mode = moteus::Mode::kStopped;
      }
      else if (cycle_count_ == 5)
      {
        //      primary_out.mode = moteus::Mode::kRezero;
      }
      else if ((cycle_count_ > 5) && (cycle_count_ < 7))
      {
        for (auto &cmd : *output)
        {
          // We start everything with a stopped command to clear faults.
          cmd.mode = moteus::Mode::kStopped;
        }
      }
      else
      {
        // Then we make the secondary servo mirror the primary servo.
        const auto primary = Get(status, arguments_.primary_id, arguments_.primary_bus);
        double primary_pos = primary.position;
        if (!std::isnan(primary_pos) && std::isnan(primary_initial_))
        {
          primary_initial_ = primary_pos;
        }

        if (!std::isnan(primary_initial_))
        {
          primary_out.mode = moteus::Mode::kPosition;
          primary_out.position.position = primary_initial_ + reqPos; // secondary_initial_ + (primary_pos - primary_initial_);
          // primary_out.position.velocity = reqSpeed;//primary.velocity;
        }
      }
    }

  private:
    const Arguments arguments_;
    uint64_t cycle_count_ = 0;
    double primary_initial_ = std::numeric_limits<double>::quiet_NaN();
    double secondary_initial_ = std::numeric_limits<double>::quiet_NaN();
  };

  double Fxscale(double valueIn, double baseMin, double baseMax, double limitMin, double limitMax)
  {
    return ((limitMax - limitMin) * (valueIn - baseMin) / (baseMax - baseMin)) + limitMin;
  }
  template <typename Controller>
  void Run(const Arguments &args, Controller *controller)
  {
    if (args.help)
    {
      DisplayUsage();
      return;
    }

    moteus::ConfigureRealtime(args.main_cpu);
    MoteusInterface::Options moteus_options;
    moteus_options.cpu = args.can_cpu;
    MoteusInterface moteus_interface{moteus_options};

    std::vector<MoteusInterface::ServoCommand> commands;
    for (const auto &pair : controller->servo_bus_map())
    {
      commands.push_back({});
      commands.back().id = pair.first;
      commands.back().bus = pair.second;
    }

    std::vector<MoteusInterface::ServoReply> replies{commands.size()};
    std::vector<MoteusInterface::ServoReply> saved_replies;

    controller->Initialize(&commands);

    MoteusInterface::Data moteus_data;
    moteus_data.commands = {commands.data(), commands.size()};
    moteus_data.replies = {replies.data(), replies.size()};

    std::future<MoteusInterface::Output> can_result;

    const auto period =
        std::chrono::microseconds(static_cast<int64_t>(args.period_s * 1e6));
    auto next_cycle = std::chrono::steady_clock::now() + period;

    const auto status_period = std::chrono::milliseconds(200);
    auto next_status = next_cycle + status_period;
    uint64_t cycle_count = 0;
    double total_margin = 0.0;
    uint64_t margin_cycles = 0;
    float computedPosition = 0.0f;
    float computedSpeed = 0.0f;
    ruckig::RuckigThrow<1> otg{args.period_s}; // control cycle
    ruckig::InputParameter<1> Rinput;
    ruckig::OutputParameter<1> Routput;
    ruckig::Result Rres;

    short ActualJoyValueRaw = 0;
    double ActualJoyValue = 0.0f;
    float ReqAccel = 10.0f;
    float ReqJerk = 10.0f;
    float AccelSteps = 0.01;
    float JerkSteps = 0.01;

    bool sIncreaseAccel = false;
    bool sDecreaseAccel = false;
    bool sIncreaseJerk = false;
    bool sDecreaseJerk = false;
    double  extJSpeed = args.joy_speed;
    Joystick joystick("/dev/input/js0");
    JoystickEvent Jevent;

    // Ensure that it was found and that we can use it
    if (!joystick.isFound())
    {
      printf("Joystick not found, open failed.\n");
      exit(1);
    }
    Rinput.control_interface = ruckig::ControlInterface::Position; // ruckig::ControlInterface::Velocity;

    Rinput.max_acceleration = {2.0};
    Rinput.max_jerk = {3.0};
    Rinput.max_velocity = {2.0};

    Rinput.current_position = {0.0};
    Rinput.current_velocity = {0.0};
    Rinput.current_acceleration = {0.0};

    Rinput.target_position = {0.0};
    Rinput.target_velocity = {0.0};
    Rinput.target_acceleration = {0.0};

    //Rinput.minimum_duration = 0.01f;
    // Rinput.min_velocity ={0.01f};

    // We will run at a fixed cycle time.
    while (true)
    {
      cycle_count++;
      margin_cycles++;

      if (sIncreaseAccel)
      {
        ReqAccel += AccelSteps;
        if (ReqAccel > 100)
          ReqAccel = 100;
      }
      if (sDecreaseAccel)
      {
        ReqAccel -= AccelSteps;
        if (ReqAccel < 0)
          ReqAccel = 0.0001;
      }

      if (sIncreaseJerk)
      {
        ReqJerk += JerkSteps;
        if (ReqJerk > 100)
          ReqJerk = 100;
      }
      if (sDecreaseJerk)
      {
        ReqJerk -= JerkSteps;
        if (ReqJerk < 0)
          ReqJerk = 0.0001;
      }

      {
        const auto now = std::chrono::steady_clock::now();
        if (now > next_status)
        {
          // NOTE: iomanip is not a recommended pattern.  We use it here
          // simply to not require any external dependencies, like 'fmt'.
          const auto volts = MinMaxVoltage(saved_replies);
          const std::string modes = [&]()
          {
            std::ostringstream result;
            result.precision(4);
            result << std::fixed;
            for (const auto &item : saved_replies)
            {
              result << item.id << "/"
                     << item.bus << "|"
                     << static_cast<int>(item.result.mode) << "|"
                     << item.result.position << "/  /"
                     << computedPosition << " - "
                     << computedSpeed << " - J "
                     << ActualJoyValue << " "
                     << "R: " << Rres << " ";
            }
            /*if (Rres != 0)
            {*/
              // print data
              result << "\n\033[31m" << Rinput.to_string() << "\033[0m";
              // result << "ActualJoyValue " <<ActualJoyValue << " Raw " <<ActualJoyValueRaw<<"\n";
            //}
            return result.str();
          }();
          std::cout << std::setprecision(6) << std::fixed
                    << "Cycles " << cycle_count
                    << "  margin: " << (total_margin / margin_cycles)
                    << std::setprecision(1)
                    << "  volts: " << volts.first << "/" << volts.second
                    << "  modes: " << modes
                    << "   \r";
          std::cout.flush();
          next_status += status_period;
          total_margin = 0;
          margin_cycles = 0;
        }

        int skip_count = 0;
        while (now > next_cycle)
        {
          skip_count++;
          next_cycle += period;
        }
        if (skip_count)
        {
          std::cout << "\nSkipped " << skip_count << " cycles\n";
        }
      }
      // Wait for the next control cycle to come up.
      {
        const auto pre_sleep = std::chrono::steady_clock::now();
        std::this_thread::sleep_until(next_cycle);
        const auto post_sleep = std::chrono::steady_clock::now();
        std::chrono::duration<double> elapsed = post_sleep - pre_sleep;
        total_margin += elapsed.count();
      }
      next_cycle += period;

      if (joystick.sample(&Jevent))
      {
        if (Jevent.isButton())
        {
          /* printf("Button %u is %s\n",
                  Jevent.number,
                  Jevent.value == 0 ? "up" : "down");
                  */
          if (Jevent.number == 1)
          { // circle
            if (Jevent.value == 0)
            { // up
              sIncreaseJerk = false;
            }
            else
            { // down
              sIncreaseJerk = true;
            }
          }
          if (Jevent.number == 0)
          { // circle
            if (Jevent.value == 0)
            { // up
              sDecreaseJerk = false;
            }
            else
            { // down
              sDecreaseJerk = true;
            }
          }

          if (Jevent.number == 2)
          { // circle
            if (Jevent.value == 0)
            { // up
              sIncreaseAccel = false;
            }
            else
            { // down
              sIncreaseAccel = true;
            }
          }
          if (Jevent.number == 3)
          { // circle
            if (Jevent.value == 0)
            { // up
              sDecreaseAccel = false;
            }
            else
            { // down
              sDecreaseAccel = true;
            }
          }
        }

        if (Jevent.isAxis())
        {
          // printf("Axis %u is at position %d\n", Jevent.number, Jevent.value);
          if (Jevent.number == 0)
          {
            ActualJoyValueRaw = Jevent.value;
            //ActualJoyValue = Fxscale(ActualJoyValueRaw, -32768, 32767, -0.02, +0.02);//
            ActualJoyValue = Fxscale(ActualJoyValueRaw, -32768, 32767, fabs(extJSpeed)*-1 , fabs(extJSpeed));//
            if (ActualJoyValueRaw == 0)
            {
              ActualJoyValue = 0.0f;
            }
          }
        }
      }
      if (cycle_count > 10)
      {

        Rinput.max_acceleration = {ReqAccel};
        Rinput.max_jerk = {ReqJerk};
        Rinput.max_velocity[0] = fabs(ActualJoyValue);
        // Rinput.target_velocity= {ActualJoyValue};

        if (ActualJoyValue > 0)
        {
          Rinput.target_position = {10};
        }
        else
        {
          Rinput.target_position = {-10};
        }

        if (ActualJoyValue == 0)
        {
          Rinput.control_interface = ruckig::ControlInterface::Velocity;
        }
        else
        {
          Rinput.control_interface = ruckig::ControlInterface::Position;
        }
        if (!otg.validate_input(Rinput))
        {
          printf("\nError occured , input not validated\n");
        }
        Rres = otg.update(Rinput, Routput);
        if (Rres == ruckig::Result::Working)
        {
          auto &p = Routput.new_position;
          auto &s = Routput.new_velocity;

          computedPosition = p[0];
          computedSpeed = s[0];
          Routput.pass_to_input(Rinput);
        }
        else if (Rres == ruckig::Result::ErrorTrajectoryDuration)
        {
        }
      }
      controller->RunWithAttributes(saved_replies, &commands, computedPosition, computedSpeed);

      if (can_result.valid())
      {
        // Now we get the result of our last query and send off our new
        // one.
        const auto current_values = can_result.get();

        // We copy out the results we just got out.
        const auto rx_count = current_values.query_result_size;
        saved_replies.resize(rx_count);
        std::copy(replies.begin(), replies.begin() + rx_count,
                  saved_replies.begin());
      }

      // Then we can immediately ask them to be used again.
      auto promise = std::make_shared<std::promise<MoteusInterface::Output>>();
      moteus_interface.Cycle(
          moteus_data,
          [promise](const MoteusInterface::Output &output)
          {
            // This is called from an arbitrary thread, so we just set
            // the promise value here.
            promise->set_value(output);
          });
      can_result = promise->get_future();
    }
  }
}

int main(int argc, char **argv)
{
  Arguments args({argv + 1, argv + argc});

  // Lock memory for the whole process.
  LockMemory();

  SampleController sample_controller{args};
  Run(args, &sample_controller);

  return 0;
}
