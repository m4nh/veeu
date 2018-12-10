#ifndef POSTCARD_HPP
#define POSTCARD_HPP

#include <vector>
#include <map>

//BOOST
#include <boost/algorithm/string.hpp>
#include <boost/bind.hpp>
#include <boost/asio.hpp>
#include <boost/asio/buffer.hpp>
#include <boost/thread.hpp>
#include <boost/uuid/uuid.hpp>            // uuid class
#include <boost/uuid/uuid_generators.hpp> // generators
#include <boost/uuid/uuid_io.hpp>         // streaming operators etc.
#include <boost/lexical_cast.hpp>
#include <boost/iostreams/filtering_stream.hpp>
#include <boost/iostreams/filtering_streambuf.hpp>
#include <boost/iostreams/copy.hpp>
#include <boost/iostreams/filter/gzip.hpp>


#define COMMAND_SIZE 64
typedef uint8_t byte;

namespace veeu
{

namespace postcard
{



/**
 * Message Header
 */
struct MessageHeader
{
    uint8_t CRC[4];
    uint32_t width;
    uint32_t height;
    uint32_t depth;
    uint32_t byte_per_element;
    char command[COMMAND_SIZE];

    MessageHeader()
    {
        memset(&command[0], 0, COMMAND_SIZE);

        //Control Code
        this->CRC[0] = 0; // 6;
        this->CRC[1] = 0; // 66;
        this->CRC[2] = 0; //166;
        this->CRC[3] = 0; //6;

        //Payload Size
        this->width = 0;
        this->height = 0;
        this->depth = 0;
        this->byte_per_element = 0;

        //Command field
        memset(this->command, ' ', COMMAND_SIZE);
        //std::strncpy(this->command, std::string("").c_str());
    }

    MessageHeader(std::string cmd, int w, int h, int d, int byte_per_element)
    {
        //Control Code
        this->CRC[0] = 0; // 6;
        this->CRC[1] = 0; // 66;
        this->CRC[2] = 0; //166;
        this->CRC[3] = 0; //6;
        this->setCorrectCRC();

        //Payload Size
        this->width = w;
        this->height = h;
        this->depth = d;
        this->byte_per_element = byte_per_element;

        //Command field
        memset(this->command, ' ', COMMAND_SIZE);
        std::strncpy(this->command, cmd.c_str(), cmd.size());
    }

    void setCorrectCRC()
    {
        this->CRC[0] = 6;
        this->CRC[1] = 66;
        this->CRC[2] = 166;
        this->CRC[3] = 6;
    }

    bool checkCRC()
    {
        return this->CRC[0] == 6 &&
               this->CRC[1] == 66 && this->CRC[2] == 166 && this->CRC[3] == 6;
    }

    std::string getCommand()
    {
        std::string c = std::string(this->command).substr(0, COMMAND_SIZE);
        boost::trim(c);
        return c;
    }
};

/**
 * Message Payload
 */
struct MessagePayload
{
    std::shared_ptr<char> data_ptr;
    uint32_t data_size;

    MessagePayload(uint32_t data_size) : data_size(data_size)
    {
        data_ptr = std::shared_ptr<char>(new char[data_size], std::default_delete<char[]>());
    }

    MessagePayload() : data_size(0)
    {
    }

    ~MessagePayload()
    {
    }

    void decompressPayload()
    {
        namespace io = boost::iostreams;
        try
        {
            io::filtering_ostream os;

            //Input Stream
            std::stringstream input_compressed;
            std::string eq((char *)data_ptr.get(), data_size);
            input_compressed.str(eq);

            //Output decompressed stream
            std::vector<char> decompressed_vec = std::vector<char>();
            os.push(io::gzip_decompressor());
            os.push(io::back_inserter(decompressed_vec));
            io::copy(input_compressed, os);

            //Payload replace
            data_ptr = std::shared_ptr<char>(new char[decompressed_vec.size()], std::default_delete<char[]>());
            std::copy((decompressed_vec).begin(), (decompressed_vec).end(), (char *)data_ptr.get());
            data_size = decompressed_vec.size();
        }
        catch (io::gzip_error const &ex)
        {
            //////logger->warn("Stream is uncompressed!");
        }
    }

    template <class O>
    void embodiesObject(O object)
    {
        this->data_size = object.ByteSizeLong();
        this->data_ptr = std::shared_ptr<char>(new char[this->data_size], std::default_delete<char[]>());
        object.SerializeToArray((void *)data_ptr.get(), this->data_size);
    }

    size_t size()
    {
        return data_size;
    }
};

/**
 * Message
 */
struct Message
{
    MessageHeader header;
    MessagePayload payload;
    bool void_message;

    Message()
    {
        this->void_message = true;
    }

    Message(std::string command)
    {
        this->header = MessageHeader(command, 0, 0, 0, 0);
        this->void_message = false;
    }

    std::string getCommand()
    {
        return header.getCommand();
    }

    void computeHeaderSize()
    {
        this->header.width = this->payload.data_size;
        this->header.height = 1;
        this->header.depth = 1;
        this->header.byte_per_element = 1;
    }

    template <class TYPE>
    void embodiesMatrix(Eigen::Matrix<TYPE, Eigen::Dynamic, Eigen::Dynamic> &mat)
    {
        this->header.width = mat.cols();
        this->header.height = mat.rows();
        this->header.depth = 1;
        this->header.byte_per_element = sizeof(TYPE);

        this->payload = MessagePayload(mat.rows() * mat.cols() * sizeof(TYPE));
        Eigen::Map<Eigen::Matrix<TYPE, Eigen::Dynamic, Eigen::Dynamic>> map((TYPE *)this->payload.data_ptr.get(), mat.rows(), mat.cols());
        map = mat;
    }

    template <class TYPE>
    void fetchMatrix(Eigen::Matrix<TYPE, Eigen::Dynamic, Eigen::Dynamic> &mat)
    {
        mat = Eigen::Matrix<TYPE, Eigen::Dynamic, Eigen::Dynamic>(
            this->header.height,
            this->header.width);

        mat = Eigen::Map<Eigen::Matrix<TYPE, Eigen::Dynamic, Eigen::Dynamic>>(
            (TYPE *)this->payload.data_ptr.get(),
            this->header.height,
            this->header.width);
    }
    
    static void sendMessageToSocket(Message &message, boost::asio::ip::tcp::socket &socket)
    {
        int header_size = sizeof(message.header);
        auto header_buffer = boost::asio::buffer(&message.header, header_size);
        boost::system::error_code error;
        socket.write_some(boost::asio::buffer(header_buffer, header_size), error);

        size_t payload_size = message.payload.size();
        if (payload_size > 0)
        {
            auto payload_buffer = boost::asio::buffer(message.payload.data_ptr.get(), payload_size);
            socket.write_some(boost::asio::buffer(payload_buffer, payload_size), error);
        }
    }

    static Message receiveMessageFromSocket(boost::asio::ip::tcp::socket &socket)
    {
        // New Message
        Message message;

        // Receive hedaer
        int header_size = sizeof(message.header);
        auto header_buffer = boost::asio::buffer(&message.header, header_size);
        int byte_read = 0;
        while (byte_read < header_size)
        {
            byte_read += boost::asio::read(socket, header_buffer); // It has to read all buffer! the while is POINTLESS!
        }

        ////logger->debug("Header received. Bytes:{}, Command:'{}'", byte_read, message.header.getCommand());
        ////logger->debug("Header data fields. Width:{}, Height:{}, Depth:{}, Bpe:{}",
                    //  message.header.width,
                    //  message.header.height,
                    //  message.header.depth,
                    //  message.header.byte_per_element);

        // Header CRC check
        if (message.header.checkCRC())
        {
            ////logger->debug("Header CRC is ok!");
        }
        else
        {
            ////logger->error("Header CRC is invalid!");
            return Message();
        }

        // Receive Payload
        int payload_size = message.header.width * message.header.height * message.header.depth * message.header.byte_per_element;
        //logger->debug("Waiting for payload [{}]", payload_size);

        // Payload creation
        message.payload = MessagePayload(payload_size);
        auto payload_buffer = boost::asio::buffer(message.payload.data_ptr.get(), payload_size);

        // Continuous receive
        byte_read = 0;
        while (byte_read < payload_size)
        {
            byte_read += boost::asio::read(socket, payload_buffer);
        }
        //logger->debug("Payload received. Bytes:{}", byte_read);
        message.payload.decompressPayload();

        //logger->debug("Payload decompressed. Bytes:{}", message.payload.data_size);
        return message;
    }
};

/**
 * SESSION
 */
using boost::asio::ip::tcp;
class Session
{

  public:
    typedef std::shared_ptr<Session> Ptr;
    typedef boost::function<Message(Message)> NewMessageCallback;
    typedef boost::function<Message(std::string, Message)> NewIdentifiedMessageCallback;

    Session(boost::asio::io_service &io_service)
        : socket_(io_service), active(false)
    {
        uuid_ = boost::lexical_cast<std::string>(boost::uuids::random_generator()());
        new_message_callback_ = 0;
        new_identified_message_callback_ = 0;
    }

    ~Session()
    {
    }

    tcp::socket &getSocket()
    {
        return socket_;
    }

    std::string getUUID()
    {
        return uuid_;
    }

    std::string getName()
    {
        return "[Session]";
    }

    void registerNewMessageCallback(NewMessageCallback cb)
    {
        this->new_message_callback_ = cb;
    }

    void registerNewIdentifiedMessageCallback(NewIdentifiedMessageCallback cb)
    {
        this->new_identified_message_callback_ = cb;
    }

    void start()
    {
        this->active = true;
        this->session_thread_ = new boost::thread(boost::bind(&Session::loop, this));
        //logger->debug("New session started [{}]", uuid_);
    }

    void stop()
    {
        this->socket_.cancel();
        this->active = false;
        //logger->debug("Session closed [{}]", uuid_);
    }

    void loop()
    {

        try
        {
            while (this->active)
            {

                //logger->debug("{}: Waiting for new header...", this->getName());

                // New Message
                Message message = Message::receiveMessageFromSocket(socket_);
                Message responseMessage = manageMessage(message);

                Message::sendMessageToSocket(responseMessage, socket_);
            }
        }
        catch (const std::exception &ex)
        {
            //logger->error("{}", ex.what());
            this->stop();
        }
    }

  private:
    std::string uuid_;
    tcp::socket socket_;
    boost::thread *session_thread_;
    bool active;
    NewMessageCallback new_message_callback_;
    NewIdentifiedMessageCallback new_identified_message_callback_;

    Message manageMessage(Message message)
    {
        if (this->new_message_callback_ != 0)
        {
            return this->new_message_callback_(message);
        }
        if (this->new_identified_message_callback_ != 0)
        {
            return this->new_identified_message_callback_(this->getUUID(), message);
        }
        return Message();

        // double *data = new double[message.header.height * message.header.width * message.header.depth];
        // auto payload_buffer = boost::asio::buffer(message.payload.data_ptr.get(), message.payload.data_size);
        // data = boost::asio::buffer_cast<double *>(payload_buffer);
        // //logger->debug("Message:{} {},{}", message.getCommand(), data[0], data[message.header.height - 1]);
    }
};

class PostcardServer
{

  public:
    typedef boost::function<void(Session::Ptr)> AcceptCallback;

    PostcardServer(short port) : port_(port)
    {
        this->acceptor_ = std::shared_ptr<tcp::acceptor>(new tcp::acceptor(io_service_, tcp::endpoint(tcp::v4(), this->port_)));
        startAccept();
    }

    void run()
    {
        this->io_service_.run();
    }

    void registerAcceptCallabck(AcceptCallback cb)
    {
        this->accept_callbacks_.push_back(cb);
    }

    Session::Ptr getSessionByUUID(std::string uuid)
    {
        if (this->sessions_.count(uuid) > 0)
        {
            return sessions_[uuid];
        }
        else
        {
            return NULL;
        }
    }

  private:
    void startAccept()
    {
        // Register new Session
        Session::Ptr new_session(new veeu::postcard::Session(io_service_));
        sessions_[new_session->getUUID()] = new_session;

        // Async wait for new connection
        acceptor_->async_accept(
            new_session->getSocket(),
            boost::bind(
                &PostcardServer::handleAccept,
                this,
                new_session,
                boost::asio::placeholders::error));
    }

    void handleAccept(std::shared_ptr<veeu::postcard::Session> new_session,
                      const boost::system::error_code &error)
    {

        printf("New connection enstablished.");
        //logger->debug("New connection enstablished.");

        // Start session
        if (!error)
        {

            for (AcceptCallback cb : accept_callbacks_)
            {
                cb(new_session);
            }
            new_session->start();
        }

        // Wait for new connection
        startAccept();
    }

    std::shared_ptr<tcp::acceptor> acceptor_;
    boost::asio::io_service io_service_;
    short port_;
    std::map<std::string, Session::Ptr> sessions_;
    std::vector<AcceptCallback> accept_callbacks_;
};

class PostcardClient
{

  public:
    PostcardClient(std::string host, short port)
    {
        this->endpoint_ = boost::asio::ip::tcp::endpoint(boost::asio::ip::address::from_string(host), port);
        this->socket_ = std::shared_ptr<boost::asio::ip::tcp::socket>(new boost::asio::ip::tcp::socket(ios_));
        this->socket_->connect(endpoint_);
    }

    Message sendMessage(Message &message)
    {
        Message::sendMessageToSocket(message, *(socket_));

        Message response = Message::receiveMessageFromSocket(*(socket_));
        return response;
    }

  private:
    boost::asio::io_service ios_;
    boost::asio::ip::tcp::endpoint endpoint_;
    std::shared_ptr<boost::asio::ip::tcp::socket> socket_;
};

} // namespace postcard

} // namespace atlas
#endif
