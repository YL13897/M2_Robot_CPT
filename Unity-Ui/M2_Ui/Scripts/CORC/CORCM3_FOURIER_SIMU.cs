using CORC;
using UnityEngine;

namespace CORC
{
    public class CORCM3_FOURIER_SIMU : CORCM3_FOURIER
    {
        string lastSent = "";

        public override void Init(string ip = "192.168.7.2", int port = 2048)
        {
            //Define state values to receive (in pre-defined order: should match CORC implementation)
            State = new FixedDictionary
            {
                ["t"] = new double[1],
                ["X"] = new double[3],
                ["dX"] = new double[3],
                ["F"] = new double[3],
                ["Command"] = new double[1],
                ["MvtProgress"] = new double[1],
                ["Contribution"] = new double[1]
            };
            State.Init(new string[] { "t", "X", "dX", "F", "Command", "MvtProgress", "Contribution" });
            Initialised = true;
        }


        //Fake send
        public override void SendCmd(string cmd, double[] parameters = null)
        {
            //Store sent command
            lastSent = cmd;
            Debug.Log("CMD: "+lastSent);
        }
        
        // Fake receive: OK last cmd send
        public override string GetCmd()
        {
            if(lastSent.Length>0)
            {
                string ret = "OK"+lastSent[2]+lastSent[3];
                Debug.Log("RET: "+ret);
                //Reset
                lastSent = "";
                return ret;
            }
            else
            {
                return "";
            }
        }
    }
}