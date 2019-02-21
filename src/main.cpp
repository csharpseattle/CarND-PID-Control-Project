#include <math.h>
#include <uWS/uWS.h>
#include <iostream>
#include <string>
#include "json.hpp"
#include "PID.h"
#include <deque>

//
// for convenience
//
using nlohmann::json;
using std::string;

//
// For converting back and forth between radians and degrees.
//
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

//
// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
//
string hasData(string s)
{
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_last_of("]");
  if (found_null != string::npos)
  {
    return "";
  }
  else if (b1 != string::npos && b2 != string::npos)
  {
    return s.substr(b1, b2 - b1 + 1);
  }

  return "";
}


int main()
{
  int steps = 0;
  uWS::Hub h;

  //
  // Create our PID class instance and initialize with
  // parameters obtained by twiddling.
  //
  PID pid;
  pid.Init(0.4252619924, 5.602194225, 0.0002811013542);

  //
  // handle the message coming from the sim.
  //
  h.onMessage([&pid, &steps](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
   uWS::OpCode opCode)
  {
    //
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {
      auto s = hasData(string(data).substr(0, length));

      if (s != "")
      {
        auto j = json::parse(s);

        string event = j[0].get<string>();

        if (event == "telemetry")
        {
          steps++;
          //
          // j[1] is the data JSON object.  Parse the
          // Cross track error, the speed and the current
          // steering angle from the json.
          //
          double cte   = std::stod(j[1]["cte"].get<string>());
          double speed = std::stod(j[1]["speed"].get<string>());
          double angle = std::stod(j[1]["steering_angle"].get<string>());

          //
          // update the pid with the cross track error.
          // it will tell us if we need to reset the
          // sim due to a twiddle state change.
          //
          bool shouldReset = pid.UpdateError(cte);

          if (shouldReset == true)
          {
            //
            // We are twiddling and need to reset.
            // first, send the message to reset the sim.
            //
            string msg = "42[\"reset\",{}]";
            ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);

            //
            // then tell the pid controller that we are resetting.
            // this gives the PID controller a chance to reset its
            // array of CTEs, number of steps, and Integral sum.
            //
            pid.Reset();
          }
          else
          {
            //
            // Otherwise we are either not twiddling or
            // we are gathering CTEs in a twiddle state.
            //
            // The PID controller tells us our new steering value
            // between -1 and 1.
            //
            double steer_value = pid.TotalError();

            //
            // it also tells us our speed, currently always .30
            //
            double throttle    = pid.getSpeed();

            //
            // Package it up and send to the sim
            //
            json msgJson;
            msgJson["steering_angle"] = steer_value;
            msgJson["throttle"] = throttle;
            auto msg = "42[\"steer\"," + msgJson.dump() + "]";
            ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          }
        }
      }
      else
      {
        //
        // Manual driving
        //
        string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket message if
  }); // end h.onMessage


  h.onConnection([](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });


  h.onDisconnection([](uWS::WebSocket<uWS::SERVER> ws, int code,
   char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port))
  {
    std::cout << "Listening to port " << port << std::endl;
  }
  else
  {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }

  h.run();
}