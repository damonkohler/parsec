/*
 * Copyright (C) 2011 Google Inc.
 * 
 * Licensed under the Apache License, Version 2.0 (the "License"); you may not
 * use this file except in compliance with the License. You may obtain a copy of
 * the License at
 * 
 * http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations under
 * the License.
 */

package org.ros.android.wallclockrelay;

import org.ros.node.DefaultNodeFactory;
import org.ros.node.Node;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMain;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;
import org.ros.message.Time;
import org.ros.message.Duration;
import org.ros.message.MessageListener;

public class WallclockRelay implements NodeMain {

    Node node;
    Subscriber<org.ros.message.std_msgs.Time> referenceClock;
    Publisher<org.ros.message.std_msgs.Time> clockRepublisher;
    Duration timeOffset;
    Time lastPublishTime;

    public WallclockRelay() {
        lastPublishTime = new Time();
        timeOffset = new Duration(0.0);
    }

    @Override
    public void main(NodeConfiguration configuration) throws Exception {
        node = new DefaultNodeFactory().newNode("wall_clock_relay", configuration);

        referenceClock = node.newSubscriber("~wall_clock", "std_msgs/Time", 
                new MessageListener<org.ros.message.std_msgs.Time>() {
            @Override
            public void onNewMessage(org.ros.message.std_msgs.Time message) {
                timeOffset = message.data.subtract(node.getCurrentTime());
                publishTime();
            }});
        clockRepublisher = node.newPublisher("/wall_clock", "std_msgs/Time");

        // Update once per minute per default
        Duration republishPeriod = new Duration(node.newParameterTree().getDouble("~publish_period", 10));
        Duration pollPeriod = new Duration(0.1);

        int numSubscribers = clockRepublisher.getNumberOfSubscribers();
        while(true) {
            if(clockRepublisher.getNumberOfSubscribers() != numSubscribers) {
                // Whenever the number of subscribers increased, re-publish the current time to make sure everyone 
                // gets the newest time asap
                if(clockRepublisher.getNumberOfSubscribers() > numSubscribers)
                    publishTime();
                numSubscribers = clockRepublisher.getNumberOfSubscribers();
            } else if(republishPeriod.compareTo(node.getCurrentTime().subtract(lastPublishTime))  < 0) {
                publishTime();
            }
            Thread.sleep(pollPeriod.totalNsecs() / 1000000);
        }
    }

    @Override
    public void shutdown() {
        node.shutdown();
    }

    private void publishTime() {
        org.ros.message.std_msgs.Time msg = node.getMessageFactory().newMessage("std_msgs/Time");
        msg.data = node.getCurrentTime().add(timeOffset);
        clockRepublisher.publish(msg);
        lastPublishTime = node.getCurrentTime();
    }
}
