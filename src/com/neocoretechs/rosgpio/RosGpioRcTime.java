package com.neocoretechs.rosgpio;

import java.io.BufferedReader;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.concurrent.ArrayBlockingQueue;

import org.ros.concurrent.CancellableLoop;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.topic.Publisher;

import com.pi4j.io.gpio.GpioController;
import com.pi4j.io.gpio.GpioFactory;
import com.pi4j.io.gpio.GpioPinDigitalInput;
import com.pi4j.io.gpio.GpioPinDigitalOutput;
import com.pi4j.io.gpio.Pin;
import com.pi4j.io.gpio.PinPullResistance;
import com.pi4j.io.gpio.PinState;
import com.pi4j.io.gpio.RaspiPin;
import com.pi4j.io.gpio.event.GpioPinDigitalStateChangeEvent;
import com.pi4j.io.gpio.event.GpioPinListenerDigital;

import diagnostic_msgs.DiagnosticStatus;
/**
 * This class uses the Pi4J and WiringPi SO to monitor the discharge time of an Rc circuit
 * 2 pins are used, one to charge the cap via an output signal, and the other to
 * determine the time it takes for an analog comparator or other sensor to change
 * For instance if pin 22 is the input recording pin and 23 the output capacitor charging pin the
 * logic is as follows:
 * 22 off
 * 23 high - cap charge
 * 22 on input pullup (should read high)
 * wait for pin change...
 * 23 off
 * wait for 22 low and record time
 * 2 pins are used to try and mitigate the switching time of slow software
 * When the event happens, a message is published to the ROS bus that contains the contents
 * of the file defined by the __alertfile:=[file] directive on the command line.
 * The topic that the DiagnosticMessage is published on is robocore/status
 * @author jg
 *
 */
public class RosGpioRcTime extends AbstractNodeMain  {

	  private static Map<Integer, Pin> cobToRas = new HashMap<Integer, Pin>();
	  static {
		  cobToRas.put(18, RaspiPin.GPIO_01);
		  cobToRas.put(23, RaspiPin.GPIO_04);
		  cobToRas.put(24, RaspiPin.GPIO_05);
		  cobToRas.put(25, RaspiPin.GPIO_06);
		  cobToRas.put(4, RaspiPin.GPIO_07);
		  cobToRas.put(17, RaspiPin.GPIO_00);
		  cobToRas.put(22, RaspiPin.GPIO_03);
	  }
	// Names on the cobbler ->   18                23                24                25 
	//{ RaspiPin.GPIO_01, RaspiPin.GPIO_04, RaspiPin.GPIO_05, RaspiPin.GPIO_06 }; // Wiring PI/PI4J
	// Names on the cobbler ->    4                17                22
	//{ RaspiPin.GPIO_07, RaspiPin.GPIO_00, RaspiPin.GPIO_03 };                   // Wiring PI/PI4J
	  
	// Queue that receives events from state changes, triggers publishing
	ArrayBlockingQueue<diagnostic_msgs.DiagnosticStatus> queue = new ArrayBlockingQueue<diagnostic_msgs.DiagnosticStatus>(1024);
	// time
	long rcMillis = 0;
	long rcAverage = 0;
	long rcAlert = 25; // time in millis deviation from running average discharge time
	// Gpio listener
	GpListener gpListener = null;
	
	  @Override
	  public void onStart(final ConnectedNode connectedNode) {
			final Publisher<diagnostic_msgs.DiagnosticStatus> statpub =
					connectedNode.newPublisher("robocore/status", diagnostic_msgs.DiagnosticStatus._TYPE);
	
			//String[] outLines = readAllLines(args.get(0), args.get(1), ";");
			// publishing loop responds when queue is ready
			connectedNode.executeCancellableLoop(new CancellableLoop() {	
				@Override
				protected void setup() {
					// take time
					rcMillis = System.currentTimeMillis();
					// construct listener and prime electron pump
					gpListener = new GpListener(connectedNode, queue, statpub);
					// dropping state to low should start feedback by triggering capacitive discharge
					gpListener.rcWrite.setState(PinState.LOW);
				}
				@Override
				protected void loop() throws InterruptedException {
					DiagnosticStatus statmsg = queue.take();
					statpub.publish(statmsg);
					Thread.sleep(1);
				}	
				
			}); // cancellable loop
	  }
	  
	  public class GpListener implements GpioPinListenerDigital {
		  private ConnectedNode node;
	      private int sequenceNumber = 0;
	      ArrayBlockingQueue<diagnostic_msgs.DiagnosticStatus> queue;
	      Publisher<diagnostic_msgs.DiagnosticStatus> statpub;
	      GpioPinDigitalInput rcRead;
	      GpioPinDigitalOutput rcWrite;
	      GpioController gpio;
	      List<String> alertLines = new ArrayList<String>();
		  public GpListener(ConnectedNode node, ArrayBlockingQueue<diagnostic_msgs.DiagnosticStatus> queue, Publisher<diagnostic_msgs.DiagnosticStatus> statpub) {
			  this.node = node;  
			  this.statpub = statpub;
			  this.queue = queue;
			  gpio = GpioFactory.getInstance();      
		      // provision gpio pin #22 as an input pin with its internal pull up resistor enabled
			  rcRead = gpio.provisionDigitalInputPin(cobToRas.get(22),  PinPullResistance.PULL_UP);
			  rcRead.addListener(this);
			  rcWrite = gpio.provisionDigitalOutputPin(cobToRas.get(23),  PinState.HIGH);
			  Map<String,String> args = node.getNodeConfiguration().getCommandLineLoader().getSpecialRemappings();	
			  if( args.get("__alertfile") != null ) {
					try {
						alertLines = readAllLines("",args.get("__alertfile"),"#");
					} catch (IOException e) { alertLines.add("Cannot find alert file for GPIO Listener"); }
			  } else {
					alertLines.add("Default GPIO event message");
			  }
		  }
		  @Override
		  public void handleGpioPinDigitalStateChangeEvent(GpioPinDigitalStateChangeEvent event) {
			  // get elapsed time since last firing
			  long rcElapsed = (System.currentTimeMillis()-rcMillis);
              // display pin state on console
              //System.out.println(" --> GPIO PIN STATE CHANGE: " + event.getPin() + " = " + event.getState() + " in "+ rcElapsed +" ave:"+rcAverage);
              rcMillis = System.currentTimeMillis();
              if( rcAverage == 0 )
            	  rcAverage = rcElapsed;
              else
            	  rcAverage = (rcElapsed + rcAverage) / 2;
              /*
				std_msgs.Header imghead = node.getTopicMessageFactory().newFromType(std_msgs.Header._TYPE);
				imghead.setSeq(sequenceNumber++);
				Time tst = node.getCurrentTime();
				imghead.setStamp(tst);
				imghead.setFrameId("0");
				*/
              if( Math.abs(rcAverage-rcElapsed) > rcAlert) {
                System.out.println(" --> GPIO PIN STATE CHANGE: " + event.getPin() + " = " + event.getState() + " in "+ rcElapsed +" ave:"+rcAverage);
				DiagnosticStatus statmsg = statpub.newMessage();
				statmsg.setName("gpiotrigger");
				statmsg.setLevel(diagnostic_msgs.DiagnosticStatus.WARN);
				//StringBuilder sb = new StringBuilder();
				statmsg.setMessage("Attention!");
				//sb.append("Achtung!!\r\n");
				// populate the list of key/value in message with the lines in alert file
				List<diagnostic_msgs.KeyValue> li = new ArrayList<diagnostic_msgs.KeyValue>();
				for(String salert : alertLines) {
					diagnostic_msgs.KeyValue kv = node.getTopicMessageFactory().newFromType(diagnostic_msgs.KeyValue._TYPE);
					kv.setKey(String.valueOf(++sequenceNumber));
					kv.setValue(salert);
					li.add(kv);
					//sb.append(salert+"\r\n");
				}
				statmsg.setValues(li);
				//statmsg.setMessage(sb.toString());
				// push event to queue
				try {
					queue.put(statmsg);
				} catch (InterruptedException e) {}
              }
		  }
		}
	  
	  	public static List<String> readAllLines(String filePath, String fileName, String commentLineDelim) throws IOException {
	    	FileReader fr = new FileReader(filePath + fileName);
	        BufferedReader bufferedReader = new BufferedReader(fr);
	        List<String> lines = new ArrayList<String>();
	        String line = null;
	        while ((line = bufferedReader.readLine()) != null) {
	        	System.out.println(line);
	            if( !line.startsWith(commentLineDelim)) {
	            	lines.add(line);
	            }
	        }
	        bufferedReader.close();
	        return lines;
	    }
	  	
	  	@Override
	  	public GraphName getDefaultNodeName() {
	  		return GraphName.of("gpiotrigger");
	  	}    			
		
}
