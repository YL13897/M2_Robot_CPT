using CORC;

namespace CORC
{
    public class CORCM3_FOURIER : CORCRobot
    {
        /// <summary>
        /// Specific class to define a CORC X2 robot object and manage communication with CORC server
        /// State dictionnary will contain X: end-effector position, 
        /// dX: end-effector velocity, F: end-effector interaction force, 
        /// t: running time of CORC server, 
        /// Command state (0:nothing, 1: calib, 2: gravity/standby, 3: Path, 4: passive jerk)
        /// Movement progress mvtID.[0-1]: e.g. 1.5 half of mvt 1
        /// User contribution: 0-1
        /// </summary>
        public override void Init(string ip = "192.168.7.2", int port = 2048)
        {
            if (Client.IsConnected())
                Client.Disconnect();

            if (Client.Connect(ip, port))
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
            else
            {
                Initialised = false;
            }
        }
    }
}