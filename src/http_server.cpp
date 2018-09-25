#include <rs_web/server_http.hpp>
#include <rs_web/client_http.hpp>

//Added for the json-example
#define BOOST_SPIRIT_THREADSAFE
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>

//Added for the default_resource example
#include <fstream>
#include <boost/filesystem.hpp>
#include <vector>
#include <algorithm>

#include <ros/package.h>

using namespace std;
//Added for the json-example:
using namespace boost::property_tree;

typedef SimpleWeb::Server<SimpleWeb::HTTP> HttpServer;
typedef SimpleWeb::Client<SimpleWeb::HTTP> HttpClient;

//Added for the default_resource example
void default_resource_send(const HttpServer &server, const shared_ptr<HttpServer::Response> &response,
                           const shared_ptr<ifstream> &ifs)
{
  //read and send 128 KB at a time
  static vector<char> buffer(131072); // Safe when server is running on one thread
  streamsize read_length;
  if((read_length = ifs->read(&buffer[0], buffer.size()).gcount()) > 0)
  {
    response->write(&buffer[0], read_length);
    if(read_length == static_cast<streamsize>(buffer.size()))
    {
      server.send(response, [&server, response, ifs](const boost::system::error_code & ec)
      {
        if(!ec)
        {
          default_resource_send(server, response, ifs);
        }
        else
        {
          cerr << "Connection interrupted" << endl;
        }
      });
    }
  }
}

int main()
{
  vector<std::string> commands_history;
  //HTTP-server at port 8080 using 1 thread
  //Unless you do more heavy non-threaded processing in the resources,
  //1 thread is usually faster than several threads
  int portNr = 5555;
  HttpServer server(portNr, 1);

  auto pkg_path = ros::package::getPath("rs_web");
  server.resource["^/robosherlock/add_new_query$"]["POST"] = [&commands_history](shared_ptr<HttpServer::Response> response, shared_ptr<HttpServer::Request> request)
  {
    try
    {
      ptree pt;
      read_json(request->content, pt);
      string name = pt.get<string>("query");
      commands_history.push_back(name);
      *response << "HTTP/1.1 200 OK\r\n"
                << "Content-Type: application/json\r\n"
                << "Content-Length: " << name.length() << "\r\n\r\n"
                << name;
    }
    catch(exception &e)
    {
      *response << "HTTP/1.1 400 Bad Request\r\nContent-Length: " << strlen(e.what()) << "\r\n\r\n" << e.what();
    }
  };

  server.resource["^/robosherlock/get_history_query$"]["POST"] = [&commands_history](shared_ptr<HttpServer::Response> response, shared_ptr<HttpServer::Request> request)
  {
    try
    {
      ptree pt;
      read_json(request->content, pt);
      string index_s = pt.get<string>("index");
      int index_i = std::stoi(index_s);
      string command="{\"item\":\"";
      if (index_i >=0 && index_i < commands_history.size()){
        command = command + commands_history[commands_history.size() - index_i - 1];
      }else{
          if (index_i <= -1){
              index_s = "-1";
          }else{
              index_s = to_string(commands_history.size() - 1);
              command = command + commands_history[0];
          }

      }
      command = command + "\",\"index\":" + index_s;
      command = command + "}";
      cout << "command= " << command << endl;
      *response << "HTTP/1.1 200 OK\r\n"
                << "Content-Type: application/json\r\n"
                << "Content-Length: " << command.length() << "\r\n\r\n"
                << command;
    }
    catch(exception &e)
    {
      *response << "HTTP/1.1 400 Bad Request\r\nContent-Length: " << strlen(e.what()) << "\r\n\r\n" << e.what();
    }
  };

  //GET-example for the path /info
  //Responds with request-information
  server.resource["^/info$"]["GET"] = [](shared_ptr<HttpServer::Response> response, shared_ptr<HttpServer::Request> request)
  {
    stringstream content_stream;
    content_stream << "<h1>Request from " << request->remote_endpoint_address << " (" << request->remote_endpoint_port << ")</h1>";
    content_stream << request->method << " " << request->path << " HTTP/" << request->http_version << "<br>";
    for(auto & header : request->header)
    {
      content_stream << header.first << ": " << header.second << "<br>";
    }

    //find length of content_stream (length received using content_stream.tellp())
    content_stream.seekp(0, ios::end);

    *response <<  "HTTP/1.1 200 OK\r\nContent-Length: " << content_stream.tellp() << "\r\n\r\n" << content_stream.rdbuf();
  };

  //GET-example for the path /match/[number], responds with the matched string in path (number)
  //For instance a request GET /match/123 will receive: 123
  server.resource["^/match/([0-9]+)$"]["GET"] = [&server](shared_ptr<HttpServer::Response> response, shared_ptr<HttpServer::Request> request)
  {
    string number = request->path_match[1];
    *response << "HTTP/1.1 200 OK\r\nContent-Length: " << number.length() << "\r\n\r\n" << number;
  };

  //Default GET-example. If no other matches, this anonymous function will be called.
  //Will respond with content in the web/-directory, and its subdirectories.
  //Default file: index.html
  //Can for instance be used to retrieve an HTML 5 client that uses REST-resources on this server
  server.default_resource["GET"] = [&server,pkg_path](shared_ptr<HttpServer::Response> response, shared_ptr<HttpServer::Request> request)
  {
    try
    {
      auto web_root_path = boost::filesystem::canonical(pkg_path+"/html");
      auto path = boost::filesystem::canonical(web_root_path / request->path);
      //Check if path is within web_root_path
      if(distance(web_root_path.begin(), web_root_path.end()) > distance(path.begin(), path.end()) ||
         !equal(web_root_path.begin(), web_root_path.end(), path.begin()))
      {
        throw invalid_argument("path must be within root path");
      }
      if(boost::filesystem::is_directory(path))
      {
        path /= "index.html";
      }
      if(!(boost::filesystem::exists(path) && boost::filesystem::is_regular_file(path)))
      {
        throw invalid_argument("file does not exist");
      }

      auto ifs = make_shared<ifstream>();
      ifs->open(path.string(), ifstream::in | ios::binary);

      if(*ifs)
      {
        ifs->seekg(0, ios::end);
        auto length = ifs->tellg();

        ifs->seekg(0, ios::beg);

        *response << "HTTP/1.1 200 OK\r\nContent-Length: " << length << "\r\n\r\n";
        default_resource_send(server, response, ifs);
      }
      else
      {
        throw invalid_argument("could not read file");
      }
    }
    catch(const exception &e)
    {
      string content = "Could not open path " + request->path + ": " + e.what();
      *response << "HTTP/1.1 400 Bad Request\r\nContent-Length: " << content.length() << "\r\n\r\n" << content;
    }
  };

  thread server_thread([&server]()
  {
    server.start();
  });
  this_thread::sleep_for(chrono::seconds(1));
  server_thread.join();
  return 0;
}
