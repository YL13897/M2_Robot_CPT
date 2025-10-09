using System;
using System.Collections;
using System.Collections.Generic;
using System.IO;
using System.Threading;
using UnityEngine;
using UnityEngine.UI;

using CORC;
using trakSTAR;


namespace MvtLogging
{
	public struct UIFlags {
        public bool NoRobot;
        public bool SetGoal;
    }
	
	public class MvtLogger
	{
		private bool readyToRecord = false;
		private bool recording = false;
		private bool recordingRunning = false;
		private Thread recordingThread = null;
		private CORCM3_FOURIER robot;
		private FLNLClient mediaPipe;
		private trakSTARSensors trakstar;
		private UIFlags flags;
		private int nbTrakstarSensors = 3;
		private Text logText = null;
		
		public StreamWriter logFileStream;
		private int nbValuesFromMediapipe = 1 + 3*8; //Nb of values to log from mediapipe server
		private string fileHeader = "#t (abs),t,X_1,X_2,X_3,dX_1,dX_2,dX_3,F_1,F_2,F_3,Cmd,MvtProgress,Contrib,S1_x,S1_y,S1_z,S1_a,S1_e,S1_r,S2_x,S2_y,S2_z,S2_a,S2_e,S2_r,S3_x,S3_y,S3_z,S3_a,S3_e,S3_r,armSide(0:left 1:right),l_eye_x,l_eye_y,l_eye_z,r_eye_x,r_eye_y,r_eye_z,l_hip_x,l_hip_y,l_hip_z,r_hip_x,r_hip_y,r_hip_z,l_sh_x,l_sh_y,l_sh_z,r_sh_x,r_sh_y,r_sh_z,el_x,el_y,el_z,wr_x,wr_y,wr_z,NoRobot,SetGoal\n"; //Ooohhh ! this is rigid and ugly!
		
		public MvtLogger(CORCM3_FOURIER r, UIFlags fl, Text log_txt=null)
		{
			logText = log_txt; //Logger textbox
			
			//UI flags
			flags = fl;
			
			//M3 robot
			robot = r;
			
			//Connection to local Dept camera python script
			mediaPipe = new FLNLClient();
			
			//trakSTAR
			trakstar = new trakSTARSensors(log_txt);
		}
		
		~MvtLogger()
		{
			Stop();
			trakstar.Close();
			mediaPipe.SendCmd("DIS".ToCharArray());
		}
		
		private void Log(string txt)
		{
			if(logText)
			{
				logText.text=txt+"\n"+logText.text;
			}
			Debug.Log(txt);
		}
		
		public bool InitSensors()
		{
			Stop();
			
			if(!mediaPipe.IsConnected())
			{ 
				if(mediaPipe.Connect("127.0.0.1", 2042)==false)
				{
					Log("MediaPipe init error.");
					return false;
				}
			}

			return trakstar.Init();
		}
		
		public bool Init(string filename)
		{
			//Init trakSTAR
			if(mediaPipe.IsConnected() && trakstar.IsInitialised() && robot.IsInitialised())
			{
				//Create file and header
				logFileStream = new StreamWriter(filename);
				logFileStream.Write(fileHeader);
				
				readyToRecord = true;
			}
			else
			{
				if(!mediaPipe.IsConnected())
					Log("MediaPipe error.");
				if(!trakstar.IsInitialised())
					Log("trakSTAR error.");
				if(!robot.IsInitialised())
					Log("Robot error.");
				readyToRecord = false;
			}
			return readyToRecord;
		}
		
		public bool Start()
		{
			if(readyToRecord)
			{
				recording = true;
				recordingRunning = true;
				
				//Start MediaPipe stream
				mediaPipe.SendCmd("STA".ToCharArray());
				
				//Start recording thread
				recordingThread = new Thread(new ThreadStart(RecordSamples));
				recordingThread.IsBackground = true;
				recordingThread.Start();
			}
			else
			{
				recording = false;
				recordingRunning = false;
			}
			return recording;
		}
		
		public void Pause()
		{
			recording = false;
		}
		
		public void Resume()
		{
			recording = true;
		}
		
		public void Stop()
		{
			readyToRecord = false;
			recording = false;
			//Stop recording thread
			recordingRunning = false;
			Thread.Sleep(100);
            if (recordingThread != null)
                recordingThread.Abort();
				
			//Stop MediaPipe stream
			mediaPipe.SendCmd("STO".ToCharArray());

			//Close file
			if (logFileStream!=null)
				logFileStream.Dispose();
		}
		
		public void SetArmSide(string s)
		{
			if(s[0]=='l' || s[0]=='L') {
				mediaPipe.SendCmd("ARL".ToCharArray());
				Log("Set left arm");
			}
			if(s[0]=='r' || s[0]=='R') {
				mediaPipe.SendCmd("ARR".ToCharArray());
				Log("Set right arm");
			}
		}
		
		private void RecordSamples()
		{
			while(recordingRunning)
			{
				if(recording)
				{
					//Write common time first
					logFileStream.Write(DateTime.Now.ToString("HH:mm:ss.fff") + ",");
					
					//Write robot data
					robot.Update(); // force update of state values
					logFileStream.Write((float)robot.State["t"][0] + ","+
										(float)robot.State["X"][0] + ","+
										(float)robot.State["X"][1] + ","+
										(float)robot.State["X"][2] + ","+
										(float)robot.State["dX"][0] + ","+
										(float)robot.State["dX"][1] + ","+
										(float)robot.State["dX"][2] + ","+
										(float)robot.State["F"][0] + ","+
										(float)robot.State["F"][1] + ","+
										(float)robot.State["F"][2] + ","+
										robot.State["Command"][0].ToString("0") + ","+
										robot.State["MvtProgress"][0].ToString("0") + ","+
										robot.State["Contribution"][0].ToString("0"));

					//Write trakSTAR sensor
					trakSTAR.Record_t[] records = new trakSTAR.Record_t[nbTrakstarSensors];
					trakstar.GetSensorsRecords(records);//update sensor records
					foreach(trakSTAR.Record_t r in records)
					{
						logFileStream.Write("," + (float)r.x + "," + (float)r.y + "," + (float)r.z + "," + (float)r.a + "," + (float)r.e + "," + (float)r.r);
					}
					
					//Write mediaPipe output
					if(mediaPipe.IsReceivedValues())
					{
						double[] vals = mediaPipe.GetReceivedValues();
						foreach(double val in vals)
						{
							logFileStream.Write("," + (float)val);
						}
					}
					else
					{
						//Write NaN instead	
						for (int i=0;i<nbValuesFromMediapipe;i++)
						{
							logFileStream.Write(",nan");
						}
					}
					
					//UI Flags
					if(flags.NoRobot)
						logFileStream.Write(",1");
					else
						logFileStream.Write(",0");
					if(flags.SetGoal)
						logFileStream.Write(",1");
					else
						logFileStream.Write(",0");
					
					logFileStream.Write("\n");
				}
				//Sleep for 5ms
				Thread.Sleep(5);
			}
		}
	}
}