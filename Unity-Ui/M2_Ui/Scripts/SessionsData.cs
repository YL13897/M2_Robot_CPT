using System;
using System.Collections;
using System.Collections.Generic;
using System.Xml;
using System.Xml.Serialization;
using System.Text;
using System.IO;

/// <summary>
/// Set of structures to store and save (to XML) activities and settings data from a session
/// </summary>
namespace SessionsData
{
    /// <summary>
    /// Allow to store on activity data
    /// </summary>
    public struct ActivityData
    {
        /// <summary>
        /// Intialise the activity of type t with assistance level a and gravity compensation g
        /// </summary>
        public ActivityData(string t, double a, double g)
        {
            arm_side = "";
            type = t;
            distance = 0;
            nb_mvts = 0;
            assistance = a;
            gravity = g;
            time_s = 0;
            start_time = DateTime.Now;
        }
        
        /// <summary>
        /// Set overall time of activity (at the end)
        /// </summary>
        public void CalculateFinalTime()
        {
            TimeSpan interval = DateTime.Now - start_time;
            time_s = (float)interval.TotalSeconds;
        }
        
        /// <summary>
        /// Current running time of the activty in seconds
        /// </summary>
        public double GetTime()
        {
            TimeSpan interval = DateTime.Now - start_time;
            return interval.TotalSeconds;
        }

        public string arm_side;
        public string type; //Command use
        public float time_s;
        DateTime start_time;
        public double distance;
        public int nb_mvts;
        public double assistance;
        public double gravity;
    }

    /// <summary>
    /// Allow to store a session containing multiple activities and basic information
    /// </summary>
    public struct SessionData
    {
        /// <summary>
        /// Create and initialise the session for patient id p
        /// </summary>
        public SessionData(string s, string armside)
        {
            subject_name = s;
            arm_side = armside;
            activities = new List<ActivityData>();
            start_time = DateTime.Now;
            distance = 0;
            nb_mvts = 0;
        }
        
        /// <summary>
        /// Add an activty (done) to the session
        /// </summary>
        public void AddActivity(ActivityData a)
        {
            a.CalculateFinalTime();
            a.arm_side=arm_side;
            activities.Add(a);
        }
        
        /// <summary>
        /// Current running time of the session in minutes
        /// </summary>
        public double GetTimeMin()
        {
            TimeSpan interval = DateTime.Now - start_time;
            return interval.TotalMinutes;
        }
        
        /// <summary>
        /// Write the session information to an XML file (filename based on date and patient id)
        /// </summary>
        public void WriteToXML()
        {
            string folder = subject_name;//"Patient"+patient_id.ToString("00");
            Directory.CreateDirectory(folder);
            string filename = folder+/*"/Patient"+patient_id.ToString("00")*/ "/"+subject_name+"_"+start_time.ToString("dd-MM-yy_HH-mm-ss");
            XmlSerializer writer = new XmlSerializer(activities.GetType());
            StreamWriter file = new StreamWriter(filename+".xml");
            writer.Serialize(file, activities);
            file.Close();
        }
        
        //int patient_id;
        string subject_name;
        string arm_side;
        DateTime start_time;
        public List<ActivityData> activities;
        public double distance; //total of session
        public int nb_mvts; //total of session
    }
}