#ifndef NETWORK_MANAGER_H
#define NETWORK_MANAGER_H

#include "Arduino.h"
#include "mongoose.h"
#include "ConfigManager.h"
#include "DiagnosticManager.h"

// Forward declarations
struct mg_connection;
struct mg_mgr;

class NetworkManager
{
public:
    NetworkManager(ConfigManager &configManager, DiagnosticManager &diagnosticManager);
    ~NetworkManager();

    // Initialization
    bool begin();
    void update();

    // UDP Communication
    bool sendUDPBytes(const uint8_t *data, size_t length);
    bool sendUDPString(const char *data);

    // Connection management
    bool isNetworkReady() const;
    bool isAgIOConnected() const;
    void setAgIOConnectionStatus(bool connected);

    // PGN handling callbacks
    using PGNHandler = void (*)(struct mg_connection *, int, void *, void *);
    using RTCMHandler = void (*)(struct mg_connection *, int, void *, void *);

    void setPGNHandler(PGNHandler handler) { pgnHandler_ = handler; }
    void setRTCMHandler(RTCMHandler handler) { rtcmHandler_ = handler; }

    // Network configuration
    void updateNetworkConfig();
    uint32_t getLocalIP() const;
    uint32_t getGatewayIP() const;
    uint32_t getBroadcastIP() const;

    // Statistics
    uint32_t getBytesSent() const { return bytesSent_; }
    uint32_t getBytesReceived() const { return bytesReceived_; }
    uint32_t getPacketsSent() const { return packetsSent_; }
    uint32_t getPacketsReceived() const { return packetsReceived_; }
    void resetStatistics();

private:
    ConfigManager &configManager_;
    DiagnosticManager &diagnosticManager_;

    // Mongoose manager and connections
    struct mg_mgr *mgr_;
    struct mg_connection *pgnListener_;
    struct mg_connection *rtcmListener_;
    struct mg_connection *agioSender_;

    // Event handlers
    PGNHandler pgnHandler_;
    RTCMHandler rtcmHandler_;

    // Network state
    bool networkReady_;
    bool agioConnected_;
    elapsedMillis agioTimeoutTimer_;
    static constexpr uint32_t AGIO_TIMEOUT = 5000; // 5 seconds

    // Statistics
    uint32_t bytesSent_;
    uint32_t bytesReceived_;
    uint32_t packetsSent_;
    uint32_t packetsReceived_;

    // CPU usage tracking
    ProcessorUsage networkUsage_;

    // Helper functions
    bool initializeEthernet();
    bool setupUDPListeners();
    bool setupAgIOSender();
    void handleNetworkEvents();
    static void pgnEventHandler(struct mg_connection *c, int ev, void *ev_data, void *fn_data);
    static void rtcmEventHandler(struct mg_connection *c, int ev, void *ev_data, void *fn_data);

    // Network utilities
    uint32_t ipArrayToUint32(const uint8_t ip[4]) const;
    void buildURLString(char *buffer, size_t bufferSize, const uint8_t ip[4], uint16_t port, const char *protocol = "udp") const;
};

// External C functions for Mongoose integration
extern "C"
{
    void ethernet_init(void);
    void mongoose_init(void);
    void mongoose_poll(void);
    extern struct mg_mgr g_mgr;
}

#endif // NETWORK_MANAGER_H