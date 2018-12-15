/* $Id: WebSvcBase.java 71 2007-11-16 13:07:14Z cianci $ */

/** \file       WebSvcBase.java
 *  \brief      
 *  \author     $Author: cianci $
 *  \version    $Revision: 71 $
 *  \date       $Date: 2007-11-16 14:07:14 +0100 (Fri, 16 Nov 2007) $
 */

import net.tinyos.message.Message;
import net.tinyos.message.MessageListener;
import net.tinyos.message.MoteIF;
import java.io.IOException;
import java.io.BufferedReader;
import java.io.InputStreamReader;

public class WebSvcBase implements MessageListener {

  final static int DEFAULT_LOCAL_GROUP = 0x7d;
  final static int BROADCAST = 0xFFFF;

  MoteIF mote;

  WebSvcBase() {
    // OK, connect to the serial forwarder and start receiving data
    mote = new MoteIF(); 
    mote.registerListener(new WebSvcInitMsg(), this);
  }

  public static void main(String[] args) {
    WebSvcBase epb = new WebSvcBase();

    try { Thread.sleep(3000); }
        catch(InterruptedException e)
        { System.err.println("INTERRUPTED: "+e); }

    //epb.mote.deregisterListener(new WebSvcInitMsg(), epb);
    //epb.mote.getSource().shutdown();
  }


  public void messageReceived(int dest_addr, Message msg) {       
    if(msg instanceof WebSvcInitMsg) {
      int source, devicetype;
      WebSvcInitMsg omsg = (WebSvcInitMsg)msg;

      source = omsg.get_Source();
      devicetype = omsg.get_DeviceType();

      System.out.println(" ++ (" + source + ") = " + devicetype);

      sendNotifMsg(source,1,1);

      // TODO: also send "NEW DEVICE" up message to vlad

        try { Thread.sleep(1000); }
            catch(InterruptedException e)
            { System.err.println("INTERRUPTED: "+e); }
        sendCmdMsg(source, "L,1,1,1,1,1,1,1,1");

        try { Thread.sleep(1000); }
            catch(InterruptedException e)
            { System.err.println("INTERRUPTED: "+e); }
        sendCmdMsg(source, "D,400,400");
    }
  }
    
  void sendNotifMsg(int dst, int notiftype, int status) {
    try {
      WebSvcNotifMsg packet = new WebSvcNotifMsg();         

      packet.set_Source((short)0x0000);
      packet.set_NotifType((short)notiftype);
      packet.set_Status((short)status);

      System.out.println(" -- (" + dst + ") = notif " 
              + notiftype + " " + status);
      mote.send(dst, packet); //MoteIF.TOS_BCAST_ADDR
      
    } catch (IOException ioe) {
      System.err.println("IOException: "+ioe);
      ioe.printStackTrace();
    }       
  }

  void sendCmdMsg(int dst, String cmd) {
    try {
      WebSvcCmdMsg packet = new WebSvcCmdMsg();         
      //char cmdarray[] = cmd.toCharArray();

      packet.set_Source((short)0x0000);
      packet.setString_Cmd(cmd);

      System.out.println(" -- (" + dst + ") cmd " + cmd );
      mote.send(dst, packet); //MoteIF.TOS_BCAST_ADDR
      
    } catch (IOException ioe) {
      System.err.println("IOException: "+ioe);
      ioe.printStackTrace();
    }       
  }
    
}
