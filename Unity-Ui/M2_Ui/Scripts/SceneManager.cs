using UnityEngine;
using UnityEngine.UI;
using System.Collections;
using System.Collections.Generic;
using UnityEngine.EventSystems;
using System;
using System.Text;
using System.IO;
using CORC;
using SessionsData;
using MvtLogging;
using Pendants;

public class SceneManager : MonoBehaviour
{
    public CORCM3_FOURIER Robot;
    public MvtLogger Logger;
    public SimplePendant Pendant;
    
    public GameObject Cursor, Arrow;
    
    private Text Status;
    private InputField InputReturnCmd;
    private Text StatusSensors;

    //Variables to manage mvt progress
    private double last_t = 0;
    private double last_mvt_nb = 0;
    private bool mvt_progress_50percent = false;
    private Vector3 last_pos;
    
    SessionData SD;
    ActivityData currentActivity;
    bool first_added_activity = true;
    private UIFlags flags;

    // Start is called before the first frame update
    void Start()
    {
        //Admin panel elements
        InputField IPInput = GameObject.Find("AdminPanel/IPInput").GetComponent<InputField>();
        IPInput.text = "192.168.7.2";
        //IPInput.text = "10.100.237.114";
        
        Button ConnectBt = GameObject.Find("AdminPanel/ConnectBt").GetComponent<Button>();
        ConnectBt.onClick.AddListener(() => { Connect(ConnectBt, IPInput); });
        
        InputField InputCmd = GameObject.Find("AdminPanel/CmdInput").GetComponent<InputField>();
        InputCmd.text = "GOCA";
        
        InputReturnCmd = GameObject.Find("AdminPanel/ReturnCmdInput").GetComponent<InputField>();
        Button CmdBt = GameObject.Find("AdminPanel/CmdBt").GetComponent<Button>();
        CmdBt.onClick.AddListener(() => { SendCommand(CmdBt, InputCmd); });
        
        Button ResetBt = GameObject.Find("AdminPanel/ResetBt").GetComponent<Button>();
        ResetBt.onClick.AddListener(() => { SendResetCommand(ResetBt); });
        
        Status = GameObject.Find("AdminPanel/Status").GetComponent<Text>();
        StatusSensors = GameObject.Find("AdminPanel/StatusSensors").GetComponent<Text>();
        
        Button InitSensorsBt = GameObject.Find("AdminPanel/InitSensorsBt").GetComponent<Button>();
        InitSensorsBt.onClick.AddListener(() => { InitSensors(InitSensorsBt); });
        
        //Session state panel
        Button StartSessionBt = GameObject.Find("StartSessionBt").GetComponent<Button>();
        StartSessionBt.onClick.AddListener(() => { StartSession(StartSessionBt); });
        
        /*Button SetGoalBt = GameObject.Find("SetGoalBt").GetComponent<Button>();
        SetGoalBt.onClick.AddListener(() => { flags.SetGoal=true; });*/
        
        Button QuitBt = GameObject.Find("QuitBt").GetComponent<Button>();
        QuitBt.onClick.AddListener(() => { Quit(); });
        
        //Control panel
        Button LockBt = GameObject.Find("ControlPanel/LockBt").GetComponent<Button>();
        LockBt.GetComponentInChildren<TMPro.TextMeshProUGUI>().text = "Lock\n∩\n▀";
        LockBt.onClick.AddListener(() => { Lock(LockBt); });
        
        Slider MassSl = GameObject.Find("ControlPanel/MassSl").GetComponent<Slider>();
        MassSl.onValueChanged.AddListener(delegate { UpdateMassSlider(MassSl.value, MassSl); });
        EventTrigger e = MassSl.GetComponentInChildren<EventTrigger>();//Messing up slider events: Unity sucks at managing events easily
        EventTrigger.Entry entry = new EventTrigger.Entry();
        entry.eventID = EventTriggerType.PointerUp;
        entry.callback.AddListener(delegate { ChangeMass(MassSl.value, MassSl); });
        e.triggers.Add(entry);
        
        Button GoGravBt = GameObject.Find("ControlBtLayout/GoGravBt").GetComponent<Button>();
        GoGravBt.onClick.AddListener(() => { GoGrav(MassSl.value); });
        
        GameObject PtsLayout = GameObject.Find("ControlPanel/PtsLayout");
        Button GoJerkBt = GameObject.Find("JerkLayout/GoJerkBt").GetComponent<Button>();
        GoJerkBt.onClick.AddListener(() => { GoJerk(PtsLayout); });
        
        Slider AssistanceSl = GameObject.Find("PathLayout/AssistanceSl").GetComponent<Slider>();
        AssistanceSl.onValueChanged.AddListener(delegate { UpdatePathAssistanceSlider(AssistanceSl.value, AssistanceSl); });
        e = AssistanceSl.GetComponentInChildren<EventTrigger>();//Messing up slider events: Unity sucks at managing events easily
        entry = new EventTrigger.Entry();
        entry.eventID = EventTriggerType.PointerUp;
        entry.callback.AddListener(delegate { ChangePathAssistance(AssistanceSl.value, AssistanceSl); });
        e.triggers.Add(entry);
        
        Button GoPathBt = GameObject.Find("PathLayout/GoPathBt").GetComponent<Button>();
        GoPathBt.onClick.AddListener(() => { GoPath(PtsLayout, AssistanceSl.value); });

        Slider T_valSl = GameObject.Find("T_val").GetComponent<Slider>();
        T_valSl.onValueChanged.AddListener(delegate { UpdatePtTimeSlider(T_valSl.value, T_valSl); });
        Slider T_pauseSl = GameObject.Find("T_pause_val").GetComponent<Slider>();
        T_pauseSl.onValueChanged.AddListener(delegate { UpdatePtPauseTimeSlider(T_pauseSl.value, T_pauseSl); });
        
        //Add pt
        string bt_path="PtsLayout/0/";
        Button AddPtBt = GameObject.Find(bt_path+"AddPtBt").GetComponent<Button>();
        AddPtBt.onClick.AddListener(() => { AddPt(AddPtBt); });
        //del pt
        Button DelPtBt = GameObject.Find(bt_path+"DelPtBt").GetComponent<Button>();
        DelPtBt.onClick.AddListener(() => { DelPt(DelPtBt); });
        
        //Disable panels
        enablePanel("SessionPanel", false);
        enablePanel("ControlPanel", false);
        
        first_added_activity = true;
    }

    // Update is called once per frame
    void Update()
    {
        Status.text = "EMU status:";
        if (Robot.IsInitialised())
        {
            //Update status text box
            Status.text += " Connected\n";
            Status.text += "\tt: " + Robot.State["t"][0].ToString("####.00") + " (" + (Robot.State["t"][0]-last_t).ToString("0.000") + ")\n";
            last_t= Robot.State["t"][0];
            Status.text += "\tX:";
            foreach(double val in Robot.State["X"])
                Status.text += val.ToString("0.000") + " \t";
            Status.text += "\n";
            Status.text += "\tdX:";
            foreach (double val in Robot.State["dX"])
                Status.text += val.ToString("0.00") + " \t";
            Status.text += "\n";
            Status.text += "\tF:";
            foreach (double val in Robot.State["F"])
                Status.text += val.ToString("00.0") + " \t";

            Status.text += "\n";
            Status.text += "Command: ";
            Status.text += Robot.State["Command"][0].ToString("0");
            Status.text += "\n";
            Status.text += "Mvt progress: ";
            Status.text += Robot.State["MvtProgress"][0].ToString("0.0");
            Status.text += "\n";
            Status.text += "Contribution: ";
            Status.text += Robot.State["Contribution"][0].ToString("0.0");
            Status.text += "\n";
            
            /*Update UI flags
            if(GameObject.Find("NoRobot").GetComponent<Toggle>().isOn)
                flags.NoRobot = true;
            else
                flags.NoRobot = false;
            flags.SetGoal = false; //reset to false if it has been set*/
            
            //Update session data: mvt counts, distance...
            updateState();
            
            //Update UI
            updateContribution(Robot.State["Contribution"][0]);
            updatePtsProgress(Robot.State["MvtProgress"][0]);
            updateSessionPanel();

            //Check and manage Pendant inputs (buttons)
            updatePendant();


            //Map cursor position and force interaction vector to current robot values
            float scale = 1000;
            Vector3 Origin = new Vector3(0, 80, -500);
            Cursor.transform.position = new Vector3((float)Robot.State["X"][1], (float)Robot.State["X"][2], -(float)Robot.State["X"][0])*scale+Origin;
            Vector3 force = new Vector3((float)Robot.State["F"][1], (float)Robot.State["F"][2], -(float)Robot.State["F"][0]);
            float force_scale = 10;
            Arrow.transform.localPosition = new Vector3(0, 0, force.magnitude / force_scale);
            Arrow.transform.localScale = new Vector3(0.2f, force.magnitude / force_scale, 0.2f);
            Cursor.transform.LookAt(Cursor.transform.position - force);
        }
        else
        {
            Status.text += " Not Connected\n";
            enablePanel("ControlPanel", false);
        }
    }

    //Update relevant session data: count nv mvts, distance ...
    void updateState()
    {
        //For current activity and overall session
        //Nb mvts count
        
        if((int)last_mvt_nb == (int)Robot.State["MvtProgress"][0]) //Still in same mvt
        {
            double last_p = last_mvt_nb-(int)last_mvt_nb;
            double p = Robot.State["MvtProgress"][0]-(int)Robot.State["MvtProgress"][0];
            //Flag set true when crossing 50% of mvt: used to count future mvt
            if(!mvt_progress_50percent) {
                mvt_progress_50percent = (last_p<=0.5 && p>0.5) ? true : false;
            }
            int is_new_mvt = 0;
            //If crossing 98% while already crossed 50:
            if( (last_p<=0.999 && (p>0.999 || p<0.1)) && mvt_progress_50percent) {
                is_new_mvt = 1;
                mvt_progress_50percent = false;
                GameObject.Find("MvtSuccessSnd").GetComponent<AudioSource>().Play();
            }
            currentActivity.nb_mvts += is_new_mvt;
            SD.nb_mvts += is_new_mvt;
        }
        last_mvt_nb = Robot.State["MvtProgress"][0];
        //Distance
        Vector3 X = new Vector3((float)Robot.State["X"][0], (float)Robot.State["X"][1], (float)Robot.State["X"][2]);
        currentActivity.distance += (X-last_pos).magnitude;
        SD.distance += (X-last_pos).magnitude;
        last_pos = X;
    }
    
    //Update text info of session
    void updateSessionPanel()
    {
        if(currentActivity.type != null)
        {
            if(currentActivity.type!="None" &&  currentActivity.type!="Lock")
            {
                //Of current activity
                string act_txt =  
                currentActivity.type;
                
                if(currentActivity.type=="Deweighting")
                {
                    act_txt += " (" + currentActivity.gravity.ToString("0.0") + "kg):";
                }
                else if(currentActivity.type=="Assistive")
                {
                    act_txt += " (+" + currentActivity.assistance.ToString("0.0") + "):\t";
                }
                else
                {
                    act_txt += ":\t\t";
                }
                
                act_txt += "\t" + 
                currentActivity.GetTime().ToString("0") + "s\t" + 
                currentActivity.nb_mvts + "mvts (" + currentActivity.distance.ToString("0.0") + "m)";
                
                GameObject.Find("CurrentActivityTxt").GetComponentInChildren<TMPro.TextMeshProUGUI>().text = act_txt;
            }
            
            //of overall session
            GameObject.Find("SessionTxt").GetComponentInChildren<TMPro.TextMeshProUGUI>().text = "Session:\t" + 
            SD.GetTimeMin().ToString("0") + "min\t" + 
            SD.nb_mvts + "mvts (" + SD.distance.ToString("0.0") + "m)";
        }
        else
        {
            GameObject.Find("CurrentActivityTxt").GetComponentInChildren<TMPro.TextMeshProUGUI>().text = "";
            GameObject.Find("SessionTxt").GetComponentInChildren<TMPro.TextMeshProUGUI>().text = "Session:\t";
        }
    }

    //Check Pendant input and act accordingly
    void updatePendant()
    {
        bool[] btns=Pendant.ReadButtons();

        if(GameObject.Find("ControlPanel").GetComponent<CanvasGroup>().interactable) 
        {
            if(btns[0]) 
            {
                Debug.Log("B0");
                if(GameObject.Find("GOJETg").GetComponent<Toggle>().isOn) 
                {
                    //Nothing
                }
                else if(GameObject.Find("GOPATg").GetComponent<Toggle>().isOn)
                {
                    //Nothing
                }
                else
                {
                    //Add pt: last one
                    GameObject pts_list = GameObject.Find("PtsLayout");
                    int last_idx = pts_list.transform.childCount-1;
                    Button add_btn = GameObject.Find("PtsLayout/"+last_idx.ToString()+"/AddPtBt").GetComponent<Button>();
                    AddPt(add_btn);
                }
            }

            if(btns[1])
            {
                Debug.Log("B1");
                if(GameObject.Find("GOJETg").GetComponent<Toggle>().isOn) 
                {
                    //Stop => Deweight
                    GoGrav(GameObject.Find("ControlPanel/MassSl").GetComponent<Slider>().value);
                }
                else if(GameObject.Find("GOPATg").GetComponent<Toggle>().isOn)
                {
                    //Stop => Deweight
                    GoGrav(GameObject.Find("ControlPanel/MassSl").GetComponent<Slider>().value);
                }
                else
                {
                    //Remove pt: last one filled in
                    GameObject pts_list = GameObject.Find("PtsLayout");
                    int last_idx = pts_list.transform.childCount-1;
                    if(last_idx==0 && (GameObject.Find("PtsLayout/"+(last_idx).ToString("0")+"/VertLayout/PtLayout/"+"Hidden/x_val").GetComponent<InputField>().text == ""))
                    {
                        return;
                    }
                    else
                    {
                        if(GameObject.Find("PtsLayout/"+(last_idx).ToString("0")+"/VertLayout/PtLayout/"+"Hidden/x_val").GetComponent<InputField>().text == "")
                        {
                            last_idx--;
                            Button del_btn = GameObject.Find("PtsLayout/"+last_idx.ToString()+"/DelPtBt").GetComponent<Button>();
                            DelPt(del_btn);
                        }
                        else
                        {
                            Button del_btn = GameObject.Find("PtsLayout/"+last_idx.ToString()+"/DelPtBt").GetComponent<Button>();
                            DelPt(del_btn);
                        }
                    }

                }
            }
        }
    }
    
    //Both save current activity, create new one and populate Session panel
    void AddActivity(string name, double a, double g)
    {
        SD.AddActivity(currentActivity);
        currentActivity = new ActivityData(name, a, g);
        
        //New activity on panel
        if(name!="None" &&  name!="Lock")
        {
            GameObject list = GameObject.Find("ActivitiesList");
            TMPro.TextMeshProUGUI act = GameObject.Find("CurrentActivityTxt").GetComponentInChildren<TMPro.TextMeshProUGUI>();
            //keep and replace first existing one
            if(!first_added_activity)
            {
                TMPro.TextMeshProUGUI new_act = Instantiate(act, list.transform);
                new_act.name=SD.activities.Count.ToString("00");
                new_act.color = new Color32(255, 255, 255, 255);
            }
            else
            {
                first_added_activity = false;
            }
            //Initialise last_pos
            last_pos = new Vector3((float)Robot.State["X"][0], (float)Robot.State["X"][1], (float)Robot.State["X"][2]);
        }
    }

    void enablePanel(string panel_path, bool enable)
    {
        GameObject.Find(panel_path).GetComponent<CanvasGroup>().alpha = enable ? 1: 0.2f;
        GameObject.Find(panel_path).GetComponent<CanvasGroup>().interactable = enable;
    }
    
    void UpdatePtTimeSlider(float v, Slider sl)
    {
        sl.GetComponentsInChildren<TMPro.TextMeshProUGUI>()[1].text=v.ToString("0.0s");
    }
    
    void UpdatePtPauseTimeSlider(float v, Slider sl)
    {
        sl.GetComponentsInChildren<TMPro.TextMeshProUGUI>()[1].text=v.ToString("0s");
    }
    
    // Update the distance indicator on each point registered (delayed to wait for update when point has been deleted)
    IEnumerator UpdatePtsDistanceValues()
    {
        yield return new WaitForSeconds(0.1f);
        
        GameObject pts_list = GameObject.Find("PtsLayout");
        
        int nb_pts = 0;
        foreach (Transform pt in pts_list.transform)
        {
          //Check if valid point (w/ values)
          if(GameObject.Find("PtsLayout/"+pt.name+"/VertLayout/PtLayout/"+"Hidden/x_val").GetComponent<InputField>().text != "")
          {
              nb_pts++;
          }
        }
        
        //No active pt
        if(nb_pts==0)
        {
             GameObject.Find("PtsLayout/0/VertLayout/PtLayout/d_val").GetComponent<InputField>().text="";
        }
        //Only one pt: distance set to 0
        else if(nb_pts==1)
        {
            GameObject.Find("PtsLayout/0/VertLayout/PtLayout/d_val").GetComponent<InputField>().text="0"+"cm";
        }
        else
        {
            string vals_path_prev="";
            string vals_path="";
            for(int idx=0;idx<nb_pts;idx++)
            {
                if(idx == 0)
                {
                    //last pt is the prev pt
                    vals_path_prev = "PtsLayout/"+(nb_pts-1).ToString("0")+"/VertLayout/PtLayout/";
                }
                else
                {
                    //prev pt
                    vals_path_prev = "PtsLayout/"+(idx-1).ToString("0")+"/VertLayout/PtLayout/";
                }
                //current pt
                vals_path = "PtsLayout/"+idx.ToString("0")+"/VertLayout/PtLayout/";
                
                //prev pt
                double x0 = double.Parse(GameObject.Find(vals_path_prev+"Hidden/x_val").GetComponent<InputField
                >().text);
                double y0 = double.Parse(GameObject.Find(vals_path_prev+"Hidden/y_val").GetComponent<InputField>().text);
                double z0 = double.Parse(GameObject.Find(vals_path_prev+"Hidden/z_val").GetComponent<InputField>().text);
                //current pt
                double x1 = double.Parse(GameObject.Find(vals_path+"Hidden/x_val").GetComponent<InputField
                >().text);
                double y1 = double.Parse(GameObject.Find(vals_path+"Hidden/y_val").GetComponent<InputField>().text);
                double z1 = double.Parse(GameObject.Find(vals_path+"Hidden/z_val").GetComponent<InputField>().text);
                //Distance
                float d = Mathf.Sqrt((float)((x0-x1)*(x0-x1)+(y0-y1)*(y0-y1)+(z0-z1)*(z0-z1)))*100; //shown in cm
                GameObject.Find(vals_path+"d_val").GetComponent<InputField>().text = d.ToString("0")+"cm";
            }
        }
    }

    
    void AddPt(Button bt)
    {
        //Fill in the pt value
        string vals_path = "PtsLayout/"+bt.transform.parent.name+"/VertLayout/PtLayout/";
        GameObject.Find(vals_path+"Hidden/x_val").GetComponent<InputField>().text = Robot.State["X"][0].ToString(".000");
        GameObject.Find(vals_path+"Hidden/y_val").GetComponent<InputField>().text = Robot.State["X"][1].ToString(".000");
        GameObject.Find(vals_path+"Hidden/z_val").GetComponent<InputField>().text = Robot.State["X"][2].ToString(".000");
        
        //Update distance to previous point value
        StartCoroutine(UpdatePtsDistanceValues());
        
        //Add a new Pt if required
        int bt_idx = int.Parse(bt.transform.parent.name);
        GameObject pts_list = GameObject.Find("PtsLayout");
        
        if(pts_list.transform.childCount<6 && bt_idx+1 == pts_list.transform.childCount)
        {
            GameObject pt = GameObject.Find("PtsLayout/"+bt.transform.parent.name);
            GameObject.Find("PtsLayout/"+bt.transform.parent.name+"AddPtBt");
            GameObject new_pt = Instantiate(pt, pt.transform.parent);
            new_pt.name = (bt_idx+1).ToString("0");
            string bt_path="PtsLayout/"+new_pt.name+"/";
            Button AddPtBt = GameObject.Find(bt_path+"AddPtBt").GetComponent<Button>();
            AddPtBt.onClick.AddListener(() => { AddPt(AddPtBt); });
            Button DelPtBt = GameObject.Find(bt_path+"DelPtBt").GetComponent<Button>();
            DelPtBt.onClick.AddListener(() => { DelPt(DelPtBt); });
            //Clear values
            vals_path = "PtsLayout/"+new_pt.name+"/VertLayout/PtLayout/";
            GameObject.Find(vals_path+"Hidden/x_val").GetComponent<InputField>().text="";
            GameObject.Find(vals_path+"Hidden/y_val").GetComponent<InputField>().text="";
            GameObject.Find(vals_path+"Hidden/z_val").GetComponent<InputField>().text="";
            GameObject.Find(vals_path+"d_val").GetComponent<InputField>().text="";
            GameObject.Find(vals_path+"T_val").GetComponent<Slider>().value=4.0f;
            Slider T_valSl = GameObject.Find(vals_path+"T_val").GetComponent<Slider>();
            T_valSl.GetComponentsInChildren<TMPro.TextMeshProUGUI>()[1].text="4.0s";
            T_valSl.onValueChanged.AddListener(delegate { UpdatePtTimeSlider(T_valSl.value, T_valSl); });
            
            Slider T_pauseSl = GameObject.Find(vals_path+"T_pause_val").GetComponent<Slider>();
            T_pauseSl.value=1.0f;
            T_pauseSl.GetComponentsInChildren<TMPro.TextMeshProUGUI>()[1].text="1s";
            T_pauseSl.onValueChanged.AddListener(delegate { UpdatePtPauseTimeSlider(T_pauseSl.value, T_pauseSl); });
        }

        //Play sound
        GameObject.Find("ShortUpSnd").GetComponent<AudioSource>().Play();
    }
    
    void DelPt(Button bt)
    {
        //Remove pt (except if 1st one: clear only)
        int bt_idx = int.Parse(bt.transform.parent.name);
        GameObject pts_list = GameObject.Find("PtsLayout");
        
        //Nothing to do if last pt not keyed in yet
        if(GameObject.Find("PtsLayout/"+(bt_idx).ToString("0")+"/VertLayout/PtLayout/"+"Hidden/x_val").GetComponent<InputField>().text == "")
        {
            return; 
        }
        
        //Actually remove the point layout?
        if(pts_list.transform.childCount>1)
        {
            GameObject pt_del = GameObject.Find("PtsLayout/"+(bt_idx).ToString("0"));
            Destroy(pt_del); //Destruction will happen only at next loop
            //Rename (re-number) the remaining ones
            int n=0;
            foreach (Transform pt in pts_list.transform)
            {
              if(pt.name != bt_idx.ToString("0"))
              {
                pt.name = n.ToString("0");
                n++;
              }
            }
        }
        else
        {
            //Or just clear values if it is the first point
            string vals_path = "PtsLayout/"+"0"+"/VertLayout/PtLayout/";
            GameObject.Find(vals_path+"Hidden/x_val").GetComponent<InputField>().text="";
            GameObject.Find(vals_path+"Hidden/y_val").GetComponent<InputField>().text="";
            GameObject.Find(vals_path+"Hidden/z_val").GetComponent<InputField>().text="";
            GameObject.Find(vals_path+"T_val").GetComponent<Slider>().value=3.0f;
            GameObject.Find(vals_path+"T_pause_val").GetComponent<Slider>().value=0f;
        }
        

        //Update distance to previous point value
        StartCoroutine(UpdatePtsDistanceValues());

        //Play sound
        GameObject.Find("ShortDownSnd").GetComponent<AudioSource>().Play();
    }
    
    void updatePtsProgress(double progress)
    {
        //progress is in the form of [Mvt].percent. e.g. 1.5: half of movement from pt 1 to 2
        GameObject pts = GameObject.Find("PtsLayout");
        int idx = 0;
        foreach (Transform pt in pts.transform)
        {
            //Find pt progress slider
            Slider [] sls = pt.GetComponentsInChildren<Slider>();
            Slider progress_sl = null;
            foreach (Slider sl in sls)
            {
                if(sl.name == "ProgressSl")
                {
                    progress_sl = sl;
                }
            }
            //update it
            if(progress>idx+1) //Mvt completed
            {
                progress_sl.value=1.0f;
            }
            else if (progress>idx) //partial mvt
            {
                progress_sl.value = (float)(progress - (int)progress);
                Pendant.SetProgress(progress_sl.value);
            }
            else
            {
                progress_sl.value = 0.0f;
            }
            idx++;
        }
    }
    
    void updateContribution(double contrib)
    {   //GameObject.Find("JerkLayout/ContributionSl").GetComponent<Slider>().value=(float)contrib;
    }
    
    double [] getPts()
    {
        List<double> p = new List<double>(); //Command parameters list
        
        GameObject pts_list = GameObject.Find("PtsLayout");
        int nb_pts = 0;
        foreach (Transform pt in pts_list.transform)
        {
          //Check if valid point (w/ values)
          if(GameObject.Find("PtsLayout/"+pt.name+"/VertLayout/PtLayout/"+"Hidden/x_val").GetComponent<InputField>().text != "")
          {
              nb_pts++;
          }
        }
        //Nb pts parameter
        p.Add(nb_pts);
        //Pts values
        for (int i=0; i<nb_pts; i++ )
        {
            string pt_path="PtsLayout/"+i.ToString("0")+"/VertLayout/PtLayout/";
            //x
            InputField inp = GameObject.Find(pt_path+"Hidden/x_val").GetComponent<InputField>();
            p.Add(double.Parse(inp.text));
            //y
            inp = GameObject.Find(pt_path+"Hidden/y_val").GetComponent<InputField>();
            p.Add(double.Parse(inp.text));
            //z
            inp = GameObject.Find(pt_path+"Hidden/z_val").GetComponent<InputField>();
            p.Add(double.Parse(inp.text));
            //T
            p.Add(GameObject.Find(pt_path+"T_val").GetComponent<Slider>().value);
            //Pause T
            p.Add(GameObject.Find(pt_path+"T_pause_val").GetComponent<Slider>().value);
        }
        
        return p.ToArray();
    }

    private IEnumerator UpdateRetCmd()
    {
        yield return new WaitForSeconds(0.1f);
        InputReturnCmd.text = Robot.GetCmd();
        if(InputReturnCmd.text.Contains("OK"))
        {
            Slider MassSl = GameObject.Find("ControlPanel/MassSl").GetComponent<Slider>();
            Slider AssistanceSl = GameObject.Find("PathLayout/AssistanceSl").GetComponent<Slider>();
            GameObject.Find("CmdSuccessSnd").GetComponent<AudioSource>().Play();
            //Create new activity based on command run
            switch(InputReturnCmd.text)
            {
                //Lock/stop
                case "OKLO":
                    AddActivity("Lock", -1, -1);
                    GameObject.Find("GOLOTg").GetComponent<Toggle>().isOn = true;
                    break;
                //UnLock
                case "OKUN":
                    AddActivity("None", -1, MassSl.value);
                    Pendant.SwitchMode(mode.Deweight);
                    break;
                //Gravity
                case "OKGR":
                    AddActivity("Deweighting", -1, MassSl.value);
                    GameObject.Find("GOGRTg").GetComponent<Toggle>().isOn = true;
                    Pendant.SwitchMode(mode.Deweight);
                    break;
                //Mobilisation
                case "OKJE":
                    AddActivity("Mobilisation", -1, MassSl.value);
                    GameObject.Find("GOJETg").GetComponent<Toggle>().isOn = true;
                    Pendant.SwitchMode(mode.Mob);
                    break;
                //Path
                case "OKPA":
                    AddActivity("Assistive", AssistanceSl.value, MassSl.value);
                    GameObject.Find("GOPATg").GetComponent<Toggle>().isOn = true;
                    Pendant.SwitchMode(mode.Assist);
                    Pendant.SetProgress(.0);
                    break;
                //Reset
                case "OKRE":
                    AddActivity("Reset", -1, -1);
                    GameObject.Find("GOLOTg").GetComponent<Toggle>().isOn = true;
                    break;
                
                //Change mass
                case "OKUM":
                    currentActivity.gravity = MassSl.value;
                    break;
                //Change assistance
                case "OKUP":
                    currentActivity.assistance = AssistanceSl.value;
                    break;

                default:
                    break;
            }
        }
        else
        {
            GameObject.Find("CmdFailSnd").GetComponent<AudioSource>().Play();
        }
    }
    
    void SendCommand(Button CmdBt, InputField InputCmd)
    {
        Robot.SendCmd(InputCmd.text);
        StartCoroutine(UpdateRetCmd());
    }
    
    void SendResetCommand(Button Bt)
    {
        Robot.SendCmd("GORE");
        StartCoroutine(UpdateRetCmd());
    }
    
    void GoJerk(GameObject Pts)
    {
        Robot.SendCmd("GOJE", getPts());
        StartCoroutine(UpdateRetCmd());
    }

    void GoPath(GameObject Pts, double assist)
    {
        Robot.SendCmd("GOPA", getPts());
        StartCoroutine(UpdateRetCmd());
    }

    void GoGrav(double val)
    {
        double [] p = {val};
        Robot.SendCmd("GOGR", p);
        StartCoroutine(UpdateRetCmd());
    }

    void Lock(Button bt)
    {
        if(bt.GetComponentInChildren<TMPro.TextMeshProUGUI>().text == "Lock\n∩\n▀")
        {
            Robot.SendCmd("GOLO");
            bt.GetComponentInChildren<TMPro.TextMeshProUGUI>().text = "Unlock\n∩\n   ▀";
            StartCoroutine(UpdateRetCmd());
        }
        else
        {
            Robot.SendCmd("GOUN");
            bt.GetComponentInChildren<TMPro.TextMeshProUGUI>().text = "Lock\n∩\n▀";
            StartCoroutine(UpdateRetCmd());
        }
    }

    //Update mass value on slider WITHOUT sending the command
    void UpdateMassSlider(float v, Slider sl)
    {
        sl.GetComponentInChildren<TMPro.TextMeshProUGUI>().text=v.ToString("0.0")+"kg";
    }

    //Actually send changed mass value (on slider release only)
    void ChangeMass(float v, Slider sl)
    {
        sl.GetComponentInChildren<TMPro.TextMeshProUGUI>().text=v.ToString("0.0")+"kg";
        double [] p = {v};
        Robot.SendCmd("UDMA", p);
        StartCoroutine(UpdateRetCmd());
    }    
    
    //Update assistance value on slider WITHOUT sending the command
    void UpdatePathAssistanceSlider(float v, Slider sl)
    {
        sl.GetComponentsInChildren<TMPro.TextMeshProUGUI>()[1].text=v.ToString("0.0");
    }
    
    //Actually send changed assistance value (on slider release only)
    void ChangePathAssistance(float v, Slider sl)
    {
        sl.GetComponentsInChildren<TMPro.TextMeshProUGUI>()[1].text=v.ToString("0.0");
        double [] p = {v};
        Robot.SendCmd("UDPA", p);
        StartCoroutine(UpdateRetCmd());
    }

    void Connect(Button bt, InputField ip)
    {
        //Connect Pendant
        Pendant = new SimplePendant();
        Pendant.Connect();
        Pendant.SwitchMode(mode.None);
        Pendant.ReadButtons();//Good to clear buffer

        //Robot then sensors and logger
        if(!Robot.IsInitialised())
        {
            Robot.Init(ip.text);
            if(Robot.IsInitialised())
            {
                Logger = new MvtLogger(Robot, flags, GameObject.Find("AdminPanel/LogSensors").GetComponent<Text>());
                if(Logger.InitSensors())
                {
                    StatusSensors.color = Color.white;
                    StatusSensors.text = "Sensors: Connected";
                }
                else
                {
                    StatusSensors.color = Color.red;
                    StatusSensors.text = "Sensors: NOT connected";
                }
                GameObject.Find("ConnectSuccessSnd").GetComponent<AudioSource>().Play();
                bt.GetComponentInChildren<Text>().text = "Disconnect";
                enablePanel("SessionPanel", true);
            }
        }
        else
        {
            Robot.Disconnect();
            bt.GetComponentInChildren<Text>().text = "Connect";
            enablePanel("ControlPanel", false);
        }
    }
    
    void InitSensors(Button InitSensorsBt)
    {
        if(Logger.InitSensors())
        {
            StatusSensors.color = Color.white;
            StatusSensors.text = "Sensors: Connected";
        }
        else
        {
            StatusSensors.color = Color.red;
            StatusSensors.text = "Sensors: NOT connected";
        }
    }
    
    //Sensor start commmand coroutine: delay required
    private IEnumerator StartLoggerCmd()
    {
        yield return new WaitForSeconds(0.5f);
        Logger.Start();
    }
    
    //Patient selected and start session pressed
    void StartSession(Button bt)
    {
        //Clear session panel
        GameObject p=GameObject.Find("ActivitiesList");
        foreach(Transform child in p.transform)
        {
            Destroy(child.gameObject);
        }
        TMPro.TMP_Dropdown Dropdown = GameObject.Find("SessionPanel/PatientsList").GetComponent<TMPro.TMP_Dropdown>();
        string val = Dropdown.options[Dropdown.value].text;
        //If session already existed: save it first
        if(SD.activities!=null)
        {
            SD.WriteToXML();
            Logger.Stop();
        }
        //Get selected arm side
        string side;
        if(GameObject.Find("ArmSide/Left").GetComponent<Toggle>().isOn)
            side = "L";
        else
            side = "R";
        SD = new SessionData(val, side);
        currentActivity = new ActivityData("None", -1, -1);
        GameObject.Find("CurrentActivityTxt").GetComponentInChildren<TMPro.TextMeshProUGUI>().text = "";
        enablePanel("ControlPanel", true);
        
        //MvtLogging file TODO: embed SessionData ??
        string folder = val;//"Patient"+val.ToString("00");
        Directory.CreateDirectory(folder);
        string filename = folder+ /*"/Patient"+val.ToString("00")*/ "/"+val+"_"+DateTime.Now.ToString("dd-MM-yy_HH-mm-ss");
        if(Logger.Init(filename+".csv"))
        {
            Logger.SetArmSide(side);
            StartCoroutine(StartLoggerCmd());
            StatusSensors.color = Color.white;
            StatusSensors.text = "Sensors: Connected. Logging.";
        }
        else
        {
            StatusSensors.color = Color.red;
            StatusSensors.text = "Sensors: Failed logger init";
            Debug.Log("Init logger error");
        }
    }
    
    void Quit()
    {
        Application.Quit();
    }
    
    private IEnumerator QuitRobot()
    {
        GoGrav(.0);
        yield return new WaitForSeconds(3.0f);
        Robot.SendCmd("QUIT");
        //Robot.Disconnect();
    }
    
    void OnApplicationQuit() 
    {
        //Write session to file if exists
        if(SD.activities!=null)
        {
            SD.WriteToXML();
            Logger.Stop();
        }
        
        if (Robot.IsInitialised())
        {
            StartCoroutine(QuitRobot());
        }
    }
}
