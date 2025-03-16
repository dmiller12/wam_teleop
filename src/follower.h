#pragma once

#include <boost/asio.hpp>

#include "udp_handler.h"
#include <barrett/detail/ca_macro.h>
#include <barrett/systems/abstract/single_io.h>
#include <barrett/thread/abstract/mutex.h>
#include <barrett/units.h>

using boost::asio::ip::udp;

template <size_t DOF> class Follower : public barrett::systems::System {
    BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

  public:
    Input<jp_type> wamJPIn;
    Output<jp_type> wamJPOutput;
    explicit Follower(barrett::systems::ExecutionManager *em, char *remoteHost,
                          int rec_port = 5554, int send_port = 5555, const std::string &sysName = "Follower")
        : System(sysName), linked(false), timed_out(true), theirJp(0.0), wamJPIn(this),
          wamJPOutput(this, &jpOutputValue), udp_handler(remoteHost, send_port, rec_port) {

        if (em != NULL) {
            em->startManaging(*this);
        }
    }

    virtual ~Follower() { this->mandatoryCleanUp(); }

    bool isLinked() const { return linked; }
    void tryLink() {
        BARRETT_SCOPED_LOCK(this->getEmMutex());

        if (!timed_out) {
            linked = true;
        }
    }
    void unlink() { linked = false; }

  protected:
    typename Output<jp_type>::Value *jpOutputValue;
    int num_received;
    jp_type wamJP;
    jv_type wamJV;
    jt_type wamJT;
    Eigen::Matrix<double, DOF, 1> sendMsg;

    using ReceivedData = typename UDPHandler<DOF>::ReceivedData;

    virtual void operate() {

        wamJP = wamJPIn.getValue();
        sendMsg << wamJP;

        udp_handler.send(sendMsg);

        boost::optional<ReceivedData> received_data = udp_handler.getLatestReceived();
        auto now = std::chrono::steady_clock::now();
        if (received_data && (now - received_data->timestamp <= TIMEOUT_DURATION)) {

            theirJp = received_data->jp;
            timed_out = false;
        } else {
            timed_out = true;
        }

        if (!linked || timed_out) {
            linked = false;
            theirJp = wamJP;
        }

        jpOutputValue->setData(&theirJp);
    }

    bool linked;
    bool timed_out;
    jp_type theirJp;

  private:
    DISALLOW_COPY_AND_ASSIGN(Follower);
    std::mutex state_mutex;
    jp_type joint_positions;
    UDPHandler<DOF> udp_handler;
    const std::chrono::milliseconds TIMEOUT_DURATION = std::chrono::milliseconds(20);
};

