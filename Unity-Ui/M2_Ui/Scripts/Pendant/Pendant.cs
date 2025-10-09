using System;
using System.IO;
using System.Collections;
using System.Collections.Generic;
using System.IO.Ports;
using UnityEngine; //TO REMOVE


namespace Pendants
{
    public enum mode {Deweight, Assist, Mob, None};

    public class SimplePendant
    {
        private const int NB_BTNS = 2;
        private bool isConnected = false;
        private SerialPort port_stream;

        public SimplePendant()
        {
        }

        ~SimplePendant()
        {
            if(isConnected)
                Disconnect();
        }

        public bool Connect()
        {
            if(isConnected)
                Disconnect();

            //Look through available ones and ask if they are a Pendants
            string[] ports = SerialPort.GetPortNames();
            foreach (string port in ports)
            {
                Debug.Log(port);
                try 
                {
                    port_stream = new SerialPort(port, 115200);
                    port_stream.WriteTimeout = 100;
                    port_stream.ReadTimeout = 100;
                    port_stream.DtrEnable = true;
                    port_stream.RtsEnable = true;
                    port_stream.Open();
                    if (port_stream.IsOpen)
                    {
                        port_stream.WriteLine("?");
                        string ret = port_stream.ReadLine();
                        Debug.Log(ret);
                        if(ret=="PENDANT")
                        {
                            isConnected = true;
                            return isConnected;
                        }
                    }   
                }
                catch (IOException)
                {
                    //next
                    Debug.Log("nope");
                }
            }
            return isConnected;
        }

        public void Disconnect() 
        {
            port_stream.Close();
            isConnected = false;
        }

        public void SwitchMode(mode m)
        {
            if(isConnected)
            {
                switch(m)
                {
                    case mode.Assist:
                        port_stream.WriteLine("SA");
                        break;

                    case mode.Mob:
                        port_stream.WriteLine("SM");
                        break;

                    case mode.Deweight:
                        port_stream.WriteLine("SD");
                        break;

                    default:
                        port_stream.WriteLine("SN");
                        break;
                }
            }
        }

        public void SetProgress(double p)
        {
            if(isConnected)
            {
                char c = System.Convert.ToChar((int)(p*100));
                string s = 'P'+c.ToString();
                port_stream.WriteLine(s);
            }
        }

        public bool[] ReadButtons()
        {
            bool[] btns = new bool[NB_BTNS];
            for(int i=0; i<NB_BTNS; i++)
            {
                btns[i] = false;
            }

            if(isConnected)
            {
                string l;
                try
                {
                    l = port_stream.ReadLine();
                }
                catch(TimeoutException)
                {
                    return btns;
                }
                if(l[0]=='B')
                {
                    for(int i=1;i <=NB_BTNS; i++)
                    {
                        if(l[i]=='1')
                        {
                            btns[i-1] = true;
                        }
                    }
                }
                //Empty buffer
                port_stream.DiscardInBuffer();
            }
            return btns;
        }
    }
}