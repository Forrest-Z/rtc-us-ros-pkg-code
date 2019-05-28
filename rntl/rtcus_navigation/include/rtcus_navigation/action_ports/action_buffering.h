/*
 * action_buffering.h
 *
 *  Created on: Apr 24, 2013
 *      Author: geus2
 */

#ifndef ACTION_BUFFERING_H_
#define ACTION_BUFFERING_H_
#include <rtcus_navigation/core.h>
#include <rtcus_stamp/stamped.h>
#include <rtcus_navigation/ActionBufferingConfig.h>
#include <dynamic_reconfigure/server.h>
#include <rtcus_conversions/conversions_stamped.h>

namespace rtcus_navigation {
namespace action_ports {

template<typename ActionType>
class ActionBuffering {
	/*TO ACCEPT STAMPED ACTIONS FOR TO BE STORED AND APLIED LATER*/
public:
	typedef typename rtcus_conversions::StampedConversion<ActionType>::StampedType StampedAction;
	ActionBuffering();
	ros::Time getLastCmdTime() const;
	bool getCurrentAction(ActionType&);
	bool isEnabled() const;
private:
	std::list<StampedAction> stamped_actions_buffer_;
	ros::Subscriber stamped_cmd_vel_topic_;
	ros::Publisher action_delay_pub_;
	boost::mutex msg_lock_;
	dynamic_reconfigure::Server<ActionBufferingConfig> configure_server_;
	ActionBufferingConfig config_;
	ros::Time t_u;
	void stampedCmdvelReceived(
			const boost::shared_ptr<const StampedAction>& msg);
	ActionBufferingConfig configuration_callback(const ActionBufferingConfig& config);
};

}
}

#endif /* ACTION_BUFFERING_H_ */
