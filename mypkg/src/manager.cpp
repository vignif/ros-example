#include "mypkg/manager.hpp"
#include <jsoncpp/json/json.h>
#include <fstream>

#include <boost/beast/core.hpp>
#include <boost/beast/http.hpp>
#include <boost/beast/version.hpp>
#include <boost/asio/connect.hpp>
#include <boost/asio/ip/tcp.hpp>
/* 
#include <curlpp/cURLpp.hpp>
#include <curlpp/Easy.hpp>
#include <curlpp/Options.hpp> */

namespace beast = boost::beast; // from <boost/beast.hpp>
namespace http = beast::http;   // from <boost/beast/http.hpp>
namespace net = boost::asio;    // from <boost/asio.hpp>
using tcp = net::ip::tcp;       // from <boost/asio/ip/tcp.hpp>

Manager::Manager(const ros::NodeHandle &nh) : _nh(nh)
{
    ROS_INFO_STREAM("Created Manager");
    _service = _nh.advertiseService("CreateCity", &Manager::CreateCity, this);
    InitDatabase();
    LoadJson();
    try
    {

        auto const host = "nominatim.openstreetmap.org/search?q=47122%2C+Forli&format=geojson&limit=1";
        //auto const host = "www.google.com";
        auto const port = "80";
        auto const target = "1.0";
        int version = 1;

        // The io_context is required for all I/O
        net::io_context ioc;

        // These objects perform our I/O
        tcp::resolver resolver(ioc);
        beast::tcp_stream stream(ioc);

        // Look up the domain name
        auto const results = resolver.resolve(host, port);

        // Make the connection on the IP address we get from a lookup
        stream.connect(results);

        // Set up an HTTP GET request message
        http::request<http::string_body> req{http::verb::get, target, version};
        req.set(http::field::host, host);
        req.set(http::field::user_agent, BOOST_BEAST_VERSION_STRING);

        // Send the HTTP request to the remote host
        http::write(stream, req);

        // This buffer is used for reading and must be persisted
        beast::flat_buffer buffer;

        // Declare a container to hold the response
        http::response<http::dynamic_body> res;

        // Receive the HTTP response
        http::read(stream, buffer, res);

        // Write the message to standard out
        std::cout << res << std::endl;

        // Gracefully close the socket
        beast::error_code ec;
        stream.socket().shutdown(tcp::socket::shutdown_both, ec);

        // not_connected happens sometimes
        // so don't bother reporting it.
        //
        if (ec && ec != beast::errc::not_connected)
            throw beast::system_error{ec};

        // If we get here then the connection is closed gracefully
    }
    catch (std::exception const &e)
    {
        std::cerr << "Error: " << e.what() << std::endl;
    }
}

Manager::~Manager()
{
    ROS_DEBUG("Closing DB");
    sqlite3_close(_db);
}

void Manager::InitDatabase()
{
    auto path = ros::package::getPath("mypkg");
    std::string nameDB;
    _nh.getParam("db_name", nameDB);

    auto fullpath = path + nameDB;

    ROS_DEBUG_STREAM("Database located in: " << fullpath);

    _rc = sqlite3_open(fullpath.c_str(), &_db);

    if (_rc)
    {
        ROS_ERROR("Can't open database: %s\n", sqlite3_errmsg(_db));
    }
    else
    {
        ROS_DEBUG("Opened database successfully\n");
        InitTable();
    }
}

static int callback(void *NotUsed, int argc, char **argv, char **azColName)
{
    int i;
    for (i = 0; i < argc; i++)
    {
        printf("%s = %s\n", azColName[i], argv[i] ? argv[i] : "NULL");
    }
    printf("\n");
    return 0;
}

void Manager::InitTable()
{
    /* Create SQL statement */
    auto sql = "CREATE TABLE IF NOT EXISTS CITIES("
               "ID INT PRIMARY KEY     NOT NULL,"
               "NAME           TEXT    NOT NULL,"
               "REGION         TEXT    NOT NULL,"
               "LATITUDE       FLOAT,"
               "LONGITUDE      FLOAT );";

    /* Execute SQL statement */
    _rc = sqlite3_exec(_db, sql, callback, 0, &_zErrMsg);

    if (_rc != SQLITE_OK)
    {
        ROS_ERROR("SQL error: %s", _zErrMsg);
        sqlite3_free(_zErrMsg);
    }
    else
    {
        ROS_DEBUG("Table created successfully");
    }
}

bool Manager::CreateCity(mypkg::AddCityToRegion::Request &req,
                         mypkg::AddCityToRegion::Response &res)
{
    if (req.city_name.size() > 0 && req.postal > 0)
    {
        auto city = City(req.city_name);

        // _objects.push_back(std::make_pair(city, region));
        ShowState();
        return true;
    }

    return false;
}

void Manager::LoadJson()
{
    Json::Value root;
    Json::Reader reader;
    auto path = ros::package::getPath("mypkg");
    std::string jsonName = "/data/cities.json";

    auto fullpath = path + jsonName;

    std::ifstream ifs(fullpath); //open file example.json

    if (!reader.parse(ifs, root))
    {
        ROS_ERROR("failed to parse");
    }

    Json::Value val = root["cities"];
    for (auto i : val)
    {
        auto name = i["name"].asString();
        auto postal = i["postal"].asInt();
        _jsonEntries.push_back(std::make_pair(postal, name));
    }
}

void Manager::ShowState()
{
    for (auto obj : _objects)
    {
        ROS_INFO("City %s in Region %s", obj.first.GetName(), obj.second.GetName());
    }
}