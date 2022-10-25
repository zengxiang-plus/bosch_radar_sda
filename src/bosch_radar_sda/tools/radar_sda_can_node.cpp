#include "can_node.h"
#include <base/program.h>
#include "can_common/can_interface.h"
#include "monitor/status_reporter.h"

using namespace driver::radar;
using drive::common::base::ROSProgram;

PLUSAI_DEFINE_string(udp_recv_port0, "53021", "udp recv port0");
PLUSAI_DEFINE_string(udp_send_address0, "10.42.0.146", "udp send address0");
PLUSAI_DEFINE_string(udp_send_port0, "53023", "udp send port0");
PLUSAI_DEFINE_string(can_conn_type0, "UDP", "can0 type");
PLUSAI_DEFINE_bool(extended_id, false, "can extended_id");
PLUSAI_DEFINE_string(ifname0, "ifname0", "can0 ifname");

class ROSRadarSDACanNodeProgram : public ROSProgram{
	public:
		ROSRadarSDACanNodeProgram() : ROSProgram("bosch_radar_sda_can_node"){}

	protected:
		bool init() override {
			this->sda_can_node_.reset(new CanNode());
			if(this->sda_can_node_->Init()){
				return true;
			}else{
				return false;
			}
		}
		void go() override { this->sda_can_node_->Run(); }
		void cleanup() override { this ->sda_can_node_.reset(); }

	private:
		std::unique_ptr<CanNode> sda_can_node_;
};

int main(int argc, char** argv){
	ROSRadarSDACanNodeProgram p;
	return p.run(argc, argv);
}