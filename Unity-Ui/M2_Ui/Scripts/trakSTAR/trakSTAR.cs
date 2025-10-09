using System;
using System.Text;
using System.Runtime.InteropServices;     // DLL support
using UnityEngine;
using UnityEngine.UI;

namespace trakSTAR
{	
	[StructLayout(LayoutKind.Sequential, CharSet = CharSet.Ansi)]       
	public struct Record_t /*DOUBLE_POSITION_ANGLES_TIME_Q_RECORD*/ {
		public double	x;
		public double	y;
		public double	z;
		public double	a;
		public double	e;
		public double	r;
		public double	time;
		public ushort	quality;		
	};

	//Attempt using the native dll
	public class trakSTARSensors
	{
		private ushort ALL_SENSORS = 65535;
		private ushort maxNbSensors = 4; //Must be 4 (max to be plugged on transmitter) even if less are actually used
		private string trakSTARiniConfigFile=".\\trakSTAR3SensorsConfig.ini";
		private bool initialised=false;
		private Text logText = null;
		
		//Imported functions		
		[DllImport("ATC3DG64.DLL", CallingConvention = CallingConvention.StdCall)]
		private static extern int InitializeBIRDSystem();
		
		[DllImport("ATC3DG64.DLL", CallingConvention = CallingConvention.StdCall)]
		private static extern int RestoreSystemConfiguration([MarshalAs(UnmanagedType.LPStr)] string filename);
		
		[DllImport("ATC3DG64.DLL", CallingConvention = CallingConvention.StdCall)]
		private static extern int CloseBIRDSystem();
		
		[DllImport("ATC3DG64.DLL", CallingConvention = CallingConvention.StdCall)]
		private static extern int GetAsynchronousRecord(ushort sensorsel, [In, Out] Record_t[] record, int size);
		
		[DllImport("ATC3DG64.DLL", CallingConvention = CallingConvention.StdCall)]
		private static extern ulong GetSensorStatus(ushort sensorid);
		
		[DllImport("ATC3DG64.DLL", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.StdCall)]
		private static extern int GetErrorText(int errorCode, StringBuilder mess, int bufferSize, int message_type/*enum: 0, 1 ??*/);
		
		public trakSTARSensors(Text log_txt=null)
		{
			logText = log_txt; //Logger textbox
		}
		
		~trakSTARSensors()
		{
			Close();
		}
		
		private void Log(string txt)
		{
			if(logText)
			{
				logText.text="trakSTAR: "+txt+"\n"+logText.text;
			}
			Debug.Log("trakSTAR: "+txt);
		}
		
		public bool Init()
		{
			initialised = false;
			Debug.Log("Init trakSTAR");
			//Init
			if(errH(InitializeBIRDSystem())=="OK") {
				//Restore saved config from file
				if(errH(RestoreSystemConfiguration(trakSTARiniConfigFile))=="OK") {
					initialised = true;
				}
			}
			return initialised;
			
	
			/*To test quickly
			Record_t[] records = new Record_t[maxNbSensors];
			int i = 0;
			while(i<100)
			{
				GetSensorsRecords(records);//update sensor records
				Debug.Log(records[0].x);
				Thread.Sleep(100);
				i++;
			}*/
		}
		
		public bool IsInitialised() 
		{
			return initialised;
		}
		
		public void Close()
		{
			errH(CloseBIRDSystem());
		}
		
		
		public bool GetSensorsRecords(Record_t[] records)
		{
			//Get record
			Record_t[] records_tmp = new Record_t[maxNbSensors];
			string ret = errH(GetAsynchronousRecord(ALL_SENSORS, records_tmp, 64*maxNbSensors));
			/*Check that all sensors status is OK: Not used as sensor 4 not used anyway
			uint status = 0;
			for(ushort i=0;i<maxNbSensors;i++)
			{
				status +=(uint)GetSensorStatus(i);
			}
			for(ushort i=0;i<maxNbSensors;i++)
			{
				Debug.Log("Status sensor "+i+": "+GetSensorStatus(i));
			}*/
			
			if(ret!="OK")
			{
				return false;
			}
			
			//Copy back values (only actual nb of sensors requested)
			for(int i=0; i<records.Length && i<maxNbSensors; i++)
			{
				records[i]=records_tmp[i];
			}
			return true;
		}
		
		//Handle error: print error message (if needed) and return error message, OK otherwise
		private string errH(int err_code)
		{
			if(err_code==0)
			{
				return new string("OK");
			}
			else
			{
				StringBuilder err_mess = new StringBuilder(255);
				GetErrorText(err_code, err_mess, 255, 0);
				Log(err_mess.ToString());
				
				return err_mess.ToString();
			}
		}
		
	}
}