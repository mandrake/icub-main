// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2007 Jonas Ruesch
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */

#include <iCub/CamCalibModule.h>
#include <yarp/math/Math.h>

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
    Bottle h_encs;
    Bottle t_encs;
    Bottle imu;
    _prtHEncsIn.read(h_encs); //head encoders
    _prtTEncsIn.read(t_encs); //torso encoders
    _prtImuIn.read(imu);   //imu data

    double t =  h_encs.get(3).asDouble()/180.0*M_PI;
    double vs = h_encs.get(4).asDouble()/180.0*M_PI;
    double vg = h_encs.get(5).asDouble()/180.0*M_PI;
    double ix = imu.get(0).asDouble()/180.0*M_PI;
    double iy = -imu.get(1).asDouble()/180.0*M_PI;
    double iz = -imu.get(2).asDouble()/180.0*M_PI;

#if 0
    double ix = h_encs.get(1).asDouble()/180.0*M_PI;
    double iy = h_encs.get(0).asDouble()/180.0*M_PI;
    double iz = h_encs.get(2).asDouble()/180.0*M_PI;

    yarp::sig::Vector neckv(3);
  
  //  Working
    neckv(0) = ix;
    neckv(1) = iy;
    neckv(2) = iz;

    //neckv(1) = ix;
    //neckv(0) = iy;
    //neckv(2) = iz;

//    neckv(1) = ix;
  //  neckv(2) = iy;
  //  neckv(0) = iz;

   // neckv(2) = ix;
   // neckv(1) = iy;
   // neckv(0) = iz;

   // neckv(0) = ix;
   // neckv(2) = iy;
   // neckv(1) = iz;

    printf ("%+5.1f %+5.1f %+5.1f", neckv(0)*180.0/M_PI,neckv(1)*180.0/M_PI,neckv(2)*180.0/M_PI);

    yarp::sig::Vector eyev(3);
    eyev(0) = 0;
    eyev(1) = t;
    if (strGroup == "CAMERA_CALIBRATION_LEFT")
    {
        eyev(2) = -(+vs + vg / 2);
    }
    else
    {
        eyev(2) = -(+vs - vg / 2);
    }

    yarp::sig::Vector neckRollv(3);
    neckRollv(0) = -ix;
    neckRollv(1) = 0;
    neckRollv(2) = 0;
    yarp::sig::Matrix neckRoll = yarp::math::rpy2dcm(neckRollv);

    yarp::sig::Vector neckPitchv(3);
    neckPitchv(0) = 0;
    neckPitchv(1) = iy;
    neckPitchv(2) = 0;
    yarp::sig::Matrix neckPitch = yarp::math::rpy2dcm(neckPitchv);

    yarp::sig::Vector neckYawv(3);
    neckYawv(0) = 0;
    neckYawv(1) = 0;
    neckYawv(2) = iz;
    yarp::sig::Matrix neckYaw = yarp::math::rpy2dcm(neckYawv);


//    yarp::sig::Matrix Rneck = yarp::math::rpy2dcm(neckv);
   // yarp::sig::Matrix Rneck = (neckPitch * neckRoll) * neckYaw;
//    yarp::sig::Matrix Rneck = (neckYaw * neckRoll) * neckPitch;
  //  yarp::sig::Matrix Rneck = (neckRoll * neckPitch) * neckYaw;
//    yarp::sig::Matrix Rneck = (neckYaw * neckPitch) * neckRoll;
  //    yarp::sig::Matrix Rneck = (neckPitch * neckYaw) * neckRoll;
//    yarp::sig::Matrix Rneck = (neckRoll * neckYaw) * neckPitch;


    yarp::sig::Matrix Rneck = (neckPitch * neckRoll) * neckYaw;
   
    yarp::sig::Matrix Reye = yarp::math::rpy2dcm(eyev);
    yarp::sig::Matrix T = Rneck*Reye;
//    yarp::sig::Matrix T = Reye*Rneck;
    yarp::sig::Vector v = yarp::math::dcm2rpy(T);

    printf (">>>>>>>> %+5.1f %+5.1f %+5.1f", v(0)*180.0/M_PI,v(1)*180.0/M_PI,v(2)*180.0/M_PI);
    printf (">>>>>>>> %+5.1f %+5.1f %+5.1f", -imu.get(0).asDouble(), imu.get(1).asDouble(), imu.get(2).asDouble());
    printf (">>>>>>>> %+5.1f %+5.1f %+5.1f\n", -imu.get(0).asDouble() - v(0)*180.0/M_PI, imu.get(1).asDouble() - v(1)*180.0/M_PI, imu.get(2).asDouble() - v(2)*180.0/M_PI);
    //WORKING
    _prtImgIn.setPose(v(0)*180.0/M_PI,v(1)*180.0/M_PI,v(2)*180.0/M_PI);
//    _prtImgIn.setPose(v(1)*180.0/M_PI,v(0)*180.0/M_PI,v(2)*180.0/M_PI);
 //   _prtImgIn.setPose(v(1)*180.0/M_PI,v(2)*180.0/M_PI,v(0)*180.0/M_PI);
//   _prtImgIn.setPose(v(2)*180.0/M_PI,v(1)*180.0/M_PI,v(0)*180.0/M_PI);
  //  _prtImgIn.setPose(v(0)*180.0/M_PI,v(2)*180.0/M_PI,v(1)*180.0/M_PI);

#endif // 0

    _prtImgIn.setPose(ix, iy, iz);

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


