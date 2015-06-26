// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2007 Jonas Ruesch
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */

#include <iCub/CamCalibModule.h>
#include <yarp/math/Math.h>
#include <yarp/os/Stamp.h>

using namespace std;
using namespace yarp::os;
using namespace yarp::math;
using namespace yarp::sig;

CamCalibPort::CamCalibPort()
{
    portImgOut=NULL;
    calibTool=NULL;

    verbose=false;
    t0=Time::now();
}

void CamCalibPort::setPointers(yarp::os::Port *_portImgOut, ICalibTool *_calibTool)
{
    portImgOut=_portImgOut;
    calibTool=_calibTool;
}

void CamCalibPort::setSaturation(double satVal)
{
    currSat = satVal;
}

void CamCalibPort::onRead(ImageOf<PixelRgb> &yrpImgIn)
{
    double t=Time::now();

    updatePose();

    // execute calibration
    if (portImgOut!=NULL)
    {        
        yarp::sig::ImageOf<PixelRgb> yrpImgOut;

        if (verbose)
            yDebug("received input image after %g [s] ... ",t-t0);

        double t1=Time::now();

        if (calibTool!=NULL)
        {
            calibTool->apply(yrpImgIn,yrpImgOut);

            for (int r =0; r <yrpImgOut.height(); r++)
            {
                for (int c=0; c<yrpImgOut.width(); c++)
                {
                    unsigned char *pixel = yrpImgOut.getPixelAddress(c,r);
                    double mean = (1.0/3.0)*(pixel[0]+pixel[1]+pixel[2]);

                    for(int i=0; i<3; i++)
                    {
                        double s=pixel[i]-mean;
                        double sn=currSat*s;
                        sn+=mean;

                        if(sn<0.0)
                            sn=0.0;
                        else if(sn>255.0)
                            sn=255.0;

                        pixel[i]=(unsigned char)sn;
                    }
                }
            }

            if (verbose)
                yDebug("calibrated in %g [s]\n",Time::now()-t1);
        }
        else
        {
            yrpImgOut=yrpImgIn;

            if (verbose)
                yDebug("just copied in %g [s]\n",Time::now()-t1);
        }

        m.lock();
        
        //timestamp propagation
        //yarp::os::Stamp stamp;
        //BufferedPort<ImageOf<PixelRgb> >::getEnvelope(stamp);
        //portImgOut->setEnvelope(stamp);

        Bottle pose;
        pose.addDouble(roll);
        pose.addDouble(pitch);
        pose.addDouble(yaw);
        portImgOut->setEnvelope(pose);

        portImgOut->write(yrpImgOut);
        m.unlock();
    }

    t0=t;
}


void CamCalibPort::updatePose() {
    yarp::os::Stamp s;
    this->getEnvelope(s);
    double time = s.getTime();


    m.lock();
    std::map<double,yarp::os::Bottle>::iterator it, it_prev, it_next;
    it_next = h_encs_map.lower_bound(time);

    it_prev = it_next;
    if(it_prev != h_encs_map.begin()) {
        --it_prev;
    }
    if(it_next == h_encs_map.end() && it_next != h_encs_map.begin()) {
        --it_next;
    }

    double diff_prev = time -it_prev->first;
    double diff_next = it_next->first - time;
    double diff = (diff_prev >= diff_next) ? diff_next : diff_prev;

    bool err_prev = ((diff_prev >= 0.0025) || (diff_prev <= -0.0025));
    bool warn_prev = ((diff_prev >= 0.0015) || (diff_prev <= -0.0015));
    bool err_next = ((diff_next >= 0.0025) || (diff_next <= -0.0025));
    bool warn_next = ((diff_next >= 0.0015) || (diff_next <= -0.0015));
    bool err = ((diff >= 0.0025) || (diff <= -0.0025));
    bool warn = ((diff >= 0.0015) || (diff <= -0.0015));
    printf("%f, %f, %s%f%s, %f, %s%f%s,               %s%f%s\n", time,
                                           it_prev->first,
                                           (err_prev ? "\033[0;31m" : (warn_prev ? "\033[0;33m" : "")), diff_prev, ((err_prev||warn_prev) ? "\033[0m" : ""),
                                           it_next->first,
                                           (err_next ? "\033[0;31m" : (warn_next ? "\033[0;33m" : "")), diff_next, ((err_next||warn_next) ? "\033[0m" : ""),
                                           (err ? "\033[0;31m" : (warn ? "\033[0;33m" : "")), diff, ((err||warn) ? "\033[0m" : ""));

    const yarp::os::Bottle& h_encs = (diff_prev >= diff_next) ? it_next->second : it_prev->second;

    double t =  h_encs.get(3).asDouble()/180.0*M_PI; // eye tilt
    double vs = h_encs.get(4).asDouble()/180.0*M_PI; // eye version
    double vg = h_encs.get(5).asDouble()/180.0*M_PI; // eye vergence

    double ix = -h_encs.get(1).asDouble()/180.0*M_PI; // neck roll
    double iy = h_encs.get(0).asDouble()/180.0*M_PI;  // neck pitch
    double iz = h_encs.get(2).asDouble()/180.0*M_PI;  // neck yaw

    double imu_x = imu.get(0).asDouble()/180.0*M_PI; // imu roll
    double imu_y = imu.get(1).asDouble()/180.0*M_PI; // imu pitch
    double imu_z = imu.get(2).asDouble()/180.0*M_PI; // imu yaw

    h_encs_map.erase(h_encs_map.begin(), it_next);
    m.unlock();

    yarp::sig::Vector neck_roll_vector(3);
    neck_roll_vector(0) = ix;
    neck_roll_vector(1) = 0;
    neck_roll_vector(2) = 0;
    yarp::sig::Matrix neck_roll_dcm = yarp::math::rpy2dcm(neck_roll_vector);

    yarp::sig::Vector neck_pitch_vector(3);
    neck_pitch_vector(0) = 0;
    neck_pitch_vector(1) = iy;
    neck_pitch_vector(2) = 0;
    yarp::sig::Matrix neck_pitch_dcm = yarp::math::rpy2dcm(neck_pitch_vector);

    yarp::sig::Vector neck_yaw_vector(3);
    neck_yaw_vector(0) = 0;
    neck_yaw_vector(1) = 0;
    neck_yaw_vector(2) = iz;
    yarp::sig::Matrix neck_yaw_dcm = yarp::math::rpy2dcm(neck_yaw_vector);

    yarp::sig::Matrix neck_dcm = (neck_pitch_dcm * neck_roll_dcm) * neck_yaw_dcm;



    yarp::sig::Vector eye_pitch_vector(3);
    eye_pitch_vector(0) = 0;
    eye_pitch_vector(1) = t;
    eye_pitch_vector(2) = 0;
    yarp::sig::Matrix eye_pitch_dcm = yarp::math::rpy2dcm(eye_pitch_vector);


    yarp::sig::Vector eye_yaw_vector(3);
    eye_yaw_vector(0) = 0;
    eye_yaw_vector(1) = 0;
    eye_yaw_vector(2) = 0;
//    if (strGroup == "CAMERA_CALIBRATION_LEFT") {
//        eye_yaw_vector(2) = -(vs + vg/2);
//    } else {
//        eye_yaw_vector(2) = -(vs - vg / 2);
//    }
    yarp::sig::Matrix eye_yaw_dcm = yarp::math::rpy2dcm(eye_yaw_vector);

    yarp::sig::Matrix eye_dcm = eye_pitch_dcm * eye_yaw_dcm;




    yarp::sig::Matrix T = neck_dcm * eye_dcm;
    yarp::sig::Vector v = yarp::math::dcm2rpy(T);





    yarp::sig::Vector imu_roll_vector(3);
    imu_roll_vector(0) = imu_x;
    imu_roll_vector(1) = 0;
    imu_roll_vector(2) = 0;
    yarp::sig::Matrix imu_roll_dcm = yarp::math::rpy2dcm(imu_roll_vector);

    yarp::sig::Vector imu_pitch_vector(3);
    imu_pitch_vector(0) = 0;
    imu_pitch_vector(1) = imu_y;
    imu_pitch_vector(2) = 0;
    yarp::sig::Matrix imu_pitch_dcm = yarp::math::rpy2dcm(imu_pitch_vector);

    yarp::sig::Vector imu_yaw_vector(3);
    imu_yaw_vector(0) = 0;
    imu_yaw_vector(1) = 0;
    imu_yaw_vector(2) = imu_z;
    yarp::sig::Matrix imu_yaw_dcm = yarp::math::rpy2dcm(imu_yaw_vector);

    yarp::sig::Matrix imu_dcm = (imu_yaw_dcm * imu_roll_dcm) * imu_pitch_dcm;

//   uncomment this to use inertial
    //yarp::sig::Vector v = yarp::math::dcm2rpy(imu_dcm);
    //v = yarp::math::dcm2rpy(imu_dcm);


#if 0
    printf("\n--------------------------------------\n");
    printf ("%+03.3f %+03.3f %+03.3f", neck_roll_vector(0)*180.0/M_PI, neck_pitch_vector(1)*180.0/M_PI, neck_yaw_vector(2)*180.0/M_PI);
    printf ("  >>>>>>>> %+03.3f %+03.3f %+03.3f", v(0)*180.0/M_PI,v(1)*180.0/M_PI,v(2)*180.0/M_PI);
    printf ("  >>>>>>>> %+03.3f %+03.3f %+03.3f", imu_r(0)*180.0/M_PI, imu_p(1)*180.0/M_PI, imu_y(2)*180.0/M_PI);

    printf("\n--------------------------------------\n");
    printf("encoders                  imu                       diff\n");
    for(int i = 0; i <= 2; ++i) {
        for(int j = 0; j <= 2; ++j) {
            printf("%+03.3f ", neck_dcm(i,j));
        }
        printf("     ");
        for(int j = 0; j <= 2; ++j) {
            printf("%+03.3f ", imu_dcm(i,j));
        }
        printf("     ");
        for(int j = 0; j <= 2; ++j) {
            double diff = neck_dcm(i,j) - imu_dcm(i,j);
            bool err = ((diff >= 0.2) || (diff <= -0.2));
            bool warn = ((diff >= 0.05) || (diff <= -0.05));
            printf("%s%+03.3f%s ", (err ? "\033[0;31m" : (warn ? "\033[0;33m" : "")), diff, ((err||warn) ? "\033[0m" : ""));
        }
        printf("\n");
    }
    printf("\n--------------------------------------\n");
#endif

    roll = v(0)*180.0/M_PI;
    pitch = v(1)*180.0/M_PI;
    yaw = v(2)*180.0/M_PI;
}


CamCalibModule::CamCalibModule(){

    _calibTool = NULL;	
}

CamCalibModule::~CamCalibModule(){

}

bool CamCalibModule::configure(yarp::os::ResourceFinder &rf){

    ConstString str = rf.check("name", Value("/camCalib"), "module name (string)").asString();
    setName(str.c_str()); // modulePortName

    // pass configuration over to bottle
    Bottle botConfig(rf.toString().c_str());
    botConfig.setMonitor(rf.getMonitor());		
    // Load from configuration group ([<group_name>]), if group option present
    Value *valGroup; // check assigns pointer to reference
    if(botConfig.check("group", valGroup, "Configuration group to load module options from (string)."))
    {
        strGroup = valGroup->asString().c_str();        
        // is group a valid bottle?
        if (botConfig.check(strGroup.c_str())){            
            Bottle &group=botConfig.findGroup(strGroup.c_str(),string("Loading configuration from group " + strGroup).c_str());
            botConfig.fromString(group.toString());
        }
        else{
            yError() << "Group " << strGroup << " not found.";
            return false;
        }
    }
    else
    {
        yError ("There seem to be an error loading parameters (group section missing), stopping module");
        return false;
    }

    string calibToolName = botConfig.check("projection",
                                         Value("pinhole"),
                                         "Projection/mapping applied to calibrated image [projection|spherical] (string).").asString().c_str();

    _calibTool = CalibToolFactories::getPool().get(calibToolName.c_str());
    if (_calibTool!=NULL) {
        bool ok = _calibTool->open(botConfig);
        if (!ok) {
            delete _calibTool;
            _calibTool = NULL;
            return false;
        }
    }

    if (yarp::os::Network::exists(getName("/in")))
    {
        yWarning() << "port " << getName("/in") << " already in use";
    }
    if (yarp::os::Network::exists(getName("/out")))
    {
        yWarning() << "port " << getName("/out") << " already in use";
    }
    if (yarp::os::Network::exists(getName("/conf")))
    {
        yWarning() << "port " << getName("/conf") << " already in use";
    }
    _prtImgIn.setSaturation(rf.check("saturation",Value(1.0)).asDouble());
    _prtImgIn.open(getName("/in"));
    _prtImgIn.setPointers(&_prtImgOut,_calibTool);
    _prtImgIn.setVerbose(rf.check("verbose"));
    _prtImgIn.useCallback();
    _prtImgOut.open(getName("/out"));
    _configPort.open(getName("/conf"));
    _prtHEncsIn.open(getName("/head_encs/in"));
    _prtTEncsIn.open(getName("/torso_encs/in"));
    _prtImuIn.open(getName("/imu/in"));
    attach(_configPort);
    fflush(stdout);

    return true;
}

bool CamCalibModule::close(){
    _prtImgIn.close();
	_prtImgOut.close();
    _prtHEncsIn.close();
    _prtTEncsIn.close();
    _configPort.close();
    if (_calibTool != NULL){
        _calibTool->close();
        delete _calibTool;
        _calibTool = NULL;
    }
    return true;
}

bool CamCalibModule::interruptModule(){
    _prtImgIn.interrupt();
    _prtImgOut.interrupt();
    _configPort.interrupt();
    return true;
}

bool CamCalibModule::updateModule()
{
    Bottle* h_encs = _prtHEncsIn.read(false); //head encoders
    Bottle* t_encs = _prtTEncsIn.read(false); //torso encoders
    Bottle* imu = _prtImuIn.read(false); //imu data

    if (h_encs!=NULL) {
        yarp::os::Stamp s;
        _prtHEncsIn.getEnvelope(s);
        double time = s.getTime();
        if(time !=0) {
            _prtImgIn.setHeadEncoders(time, *h_encs);
        }
    }

    if (t_encs!=NULL) _prtImgIn.setTorsoEncoders(*t_encs);
    if (imu!=NULL)    _prtImgIn.setImuData(*imu);

    return true;
}

double CamCalibModule::getPeriod() {
  return 0.001;
}

bool CamCalibModule::respond(const Bottle& command, Bottle& reply) 
{
    reply.clear(); 

    if (command.get(0).asString()=="quit") 
    {
        reply.addString("quitting");
        return false;     
    }
    else if (command.get(0).asString()=="sat" || command.get(0).asString()=="saturation")
    {
        double satVal = command.get(1).asDouble();
        _prtImgIn.setSaturation(satVal);
        
        reply.addString("ok");
    }
    else
    {
        yError() << "command not known - type help for more info";
    }
    return true;
}

