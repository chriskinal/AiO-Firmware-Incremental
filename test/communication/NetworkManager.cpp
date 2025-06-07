#include "NetworkManager.h"
#include "mongoose_glue.h"

NetworkManager::NetworkManager(ConfigManager &configManager, DiagnosticManager &diagnosticManager)
    : configManager_(configManager), diagnosticManager_(diagnosticManager), mgr_(&g_mgr), pgnListener_(nullptr), rtcmListener_(nullptr), agioSender_(nullptr), pgnHandler_(nullptr), rtcmHandler_(nullptr), networkReady_(false), agioConnected_(false), bytesSent_(0), bytesReceived_(0), packetsSent_(0), packetsReceived_(0), networkUsage_("NETWORK")
{
}

NetworkManager::~NetworkManager()
{
}

bool NetworkManager::begin()
{
    Serial.print("\r\n- Network Manager initialization");

    // Register CPU usage tracking
    diagnosticManager_.registerCpuUsage("NETWORK", &networkUsage_);

    // Initialize ethernet hardware
    if (!initializeEthernet())
    {
        Serial.print("\r\n  - Ethernet initialization failed");
        return false;
    }

    // Configure network settings
    updateNetworkConfig();

    // Setup UDP listeners and sender
    if (!setupUDPListeners() || !setupAgIOSender())
    {
        Serial.print("\r\n  - UDP setup failed");
        return false;
    }

    networkReady_ = true;
    Serial.print("\r\n- Network Manager initialization complete");
    return true;
}

bool NetworkManager::initializeEthernet()
{
    ethernet_init();
    mongoose_init();
    return true;
}

void NetworkManager::updateNetworkConfig()
{
    const auto &netConfig = configManager_.getNetConfig();

    // Configure Mongoose IP stack
    mgr_->ifp->enable_dhcp_client = 0;
    mgr_->ifp->ip = ipArrayToUint32(netConfig.currentIP);
    mgr_->ifp->gw = ipArrayToUint32(netConfig.gatewayIP);
    mgr_->ifp->mask = MG_IPV4(255, 255, 255, 0);

    Serial.print("\r\n  - Network configured: ");
    Serial.print(netConfig.currentIP[0]);
    Serial.print(".");
    Serial.print(netConfig.currentIP[1]);
    Serial.print(".");
    Serial.print(netConfig.currentIP[2]);
    Serial.print(".");
    Serial.print(netConfig.currentIP[3]);
}

bool NetworkManager::setupUDPListeners()
{
    const auto &netConfig = configManager_.getNetConfig();
    char pgnURL[50];
    char rtcmURL[50];

    // Build PGN listener URL (port 8888)
    buildURLString(pgnURL, sizeof(pgnURL), netConfig.currentIP, 8888);

    // Build RTCM listener URL (port 2233)
    buildURLString(rtcmURL, sizeof(rtcmURL), netConfig.currentIP, 2233);

    // Create PGN listener
    pgnListener_ = mg_listen(mgr_, pgnURL, pgnEventHandler, this);
    if (pgnListener_ == nullptr)
    {
        Serial.print("\r\n  - Failed to create PGN listener on port 8888");
        return false;
    }

    // Create RTCM listener
    rtcmListener_ = mg_listen(mgr_, rtcmURL, rtcmEventHandler, this);
    if (rtcmListener_ == nullptr)
    {
        Serial.print("\r\n  - Failed to create RTCM listener on port 2233");
        return false;
    }

    Serial.print("\r\n  - UDP listeners created (PGN:8888, RTCM:2233)");
    return true;
}

bool NetworkManager::setupAgIOSender()
{
    const auto &netConfig = configManager_.getNetConfig();
    char agioURL[30];

    // Build AgIO sender URL (broadcast to port 9999)
    buildURLString(agioURL, sizeof(agioURL), netConfig.broadcastIP, 9999);

    agioSender_ = mg_connect(mgr_, agioURL, nullptr, nullptr);
    if (agioSender_ == nullptr)
    {
        Serial.print("\r\n  - Failed to create AgIO sender connection");
        return false;
    }

    Serial.print("\r\n  - AgIO sender created (port 9999)");
    return true;
}

void NetworkManager::update()
{
    networkUsage_.timeIn();

    // Poll Mongoose for network events
    mongoose_poll();

    // Check AgIO connection timeout
    if (agioTimeoutTimer_ > AGIO_TIMEOUT && agioConnected_)
    {
        setAgIOConnectionStatus(false);
        Serial.print("\r\n  - AgIO connection timeout");
    }

    networkUsage_.timeOut();
}

bool NetworkManager::sendUDPBytes(const uint8_t *data, size_t length)
{
    if (!networkReady_ || !agioSender_ || !data || length == 0)
    {
        return false;
    }

    if (mgr_->ifp->state != MG_TCPIP_STATE_READY)
    {
        return false;
    }

    if (mg_send(agioSender_, data, length) <= 0)
    {
        return false;
    }

    // Clear send buffer to prevent accumulation
    mg_iobuf_del(&agioSender_->send, 0, agioSender_->send.len);

    // Update statistics
    bytesSent_ += length;
    packetsSent_++;

    return true;
}

bool NetworkManager::sendUDPString(const char *data)
{
    if (!data)
        return false;
    return sendUDPBytes((const uint8_t *)data, strlen(data));
}

bool NetworkManager::isNetworkReady() const
{
    return networkReady_ && (mgr_->ifp->state == MG_TCPIP_STATE_READY);
}

bool NetworkManager::isAgIOConnected() const
{
    return agioConnected_;
}

void NetworkManager::setAgIOConnectionStatus(bool connected)
{
    if (connected != agioConnected_)
    {
        agioConnected_ = connected;
        agioTimeoutTimer_ = 0;

        Serial.print("\r\n  - AgIO ");
        Serial.print(connected ? "connected" : "disconnected");
    }
}

uint32_t NetworkManager::getLocalIP() const
{
    return mgr_->ifp->ip;
}

uint32_t NetworkManager::getGatewayIP() const
{
    return mgr_->ifp->gw;
}

uint32_t NetworkManager::getBroadcastIP() const
{
    const auto &netConfig = configManager_.getNetConfig();
    return ipArrayToUint32(netConfig.broadcastIP);
}

void NetworkManager::resetStatistics()
{
    bytesSent_ = 0;
    bytesReceived_ = 0;
    packetsSent_ = 0;
    packetsReceived_ = 0;
}

uint32_t NetworkManager::ipArrayToUint32(const uint8_t ip[4]) const
{
    char buf[16];
    mg_snprintf(buf, sizeof(buf), "%d.%d.%d.%d", ip[0], ip[1], ip[2], ip[3]);
    struct mg_addr a = {};
    mg_aton(mg_str(buf), &a);
    return *(uint32_t *)&a.ip;
}

void NetworkManager::buildURLString(char *buffer, size_t bufferSize, const uint8_t ip[4], uint16_t port, const char *protocol) const
{
    mg_snprintf(buffer, bufferSize, "%s://%d.%d.%d.%d:%d",
                protocol, ip[0], ip[1], ip[2], ip[3], port);
}

// Static event handlers
void NetworkManager::pgnEventHandler(struct mg_connection *c, int ev, void *ev_data, void *fn_data)
{
    NetworkManager *self = static_cast<NetworkManager *>(fn_data);
    if (self && self->pgnHandler_)
    {
        self->pgnHandler_(c, ev, ev_data, fn_data);
    }

    // Update statistics for received data
    if (ev == MG_EV_READ && self)
    {
        self->bytesReceived_ += c->recv.len;
        self->packetsReceived_++;
    }
}

void NetworkManager::rtcmEventHandler(struct mg_connection *c, int ev, void *ev_data, void *fn_data)
{
    NetworkManager *self = static_cast<NetworkManager *>(fn_data);
    if (self && self->rtcmHandler_)
    {
        self->rtcmHandler_(c, ev, ev_data, fn_data);
    }

    // Update statistics for received data
    if (ev == MG_EV_READ && self)
    {
        self->bytesReceived_ += c->recv.len;
        self->packetsReceived_++;
    }
}