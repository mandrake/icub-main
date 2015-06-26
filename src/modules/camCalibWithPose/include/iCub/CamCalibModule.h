// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2007 Jonas Ruesch
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */

#ifndef __CAMCALIBMODULE__
#define __CAMCALIBMODULE__

 // std
#include <stdio.h>

// opencv
#include <cv.h>

// yarp
#include <yarp/os/all.h>
#include <yarp/sig/all.h>

// iCub
#include <iCub/PinholeCalibTool.h>
#include <iCub/SphericalCalibTool.h>
#include <iCub/CalibToolFactory.h>
#include <iCub/ICalibTool.h>

/**
 *
 * Camera Calibration Port class
 *
 */
class CamCalibPort : public yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> >
{
private:
    yarp::os::Port *portImgOut;
    ICalibTool     *calibTool;

    bool verbose;
    double t0;
    double currSat;
    double roll;
    double pitch;
    double yaw;
    yarp::os::Mutex m;
    double pippo;

    std::map<double, yarp::os::Bottle> h_encs_map;
//    yarp::os::Bottle h_encs ;
    yarp::os::Bottle t_encs ;
    yarp::os::Bottle imu ;

    void updatePose();

    virtual void onRead(yarp::sig::ImageOf<yarp::sig::PixelRgb> &yrpImgIn);

public:
    CamCalibPort();
    
    void setSaturation(double satVal);
    void setPointers(yarp::os::Port *_portImgOut, ICalibTool *_calibTool);
    void setVerbose(const bool sw) { verbose=sw; }

    void setHeadEncoders(double time, const yarp::os::Bottle &h_encs) { m.lock(); h_encs_map[time] = h_encs; m.unlock(); }
    void setTorsoEncoders(const yarp::os::Bottle &t_encs) { m.lock(); this->t_encs = t_encs; m.unlock(); }
    void setImuData(const yarp::os::Bottle &imu) { m.lock(); this->imu = imu; m.unlock(); }
};


class HeadEncoderPort : public yarp::os::BufferedPort<yarp::os::Bottle> {
private:
    virtual void onRead(yarp::os::Bottle &b);
public:
    CamCalibPort *_prtImgIn;
};

/**
 *
 * Camera Calibration Module class
 *
 * \see icub_camcalib
 *
 */
class CamCalibModule : public yarp::os::RFModule {

private:

    CamCalibPort    _prtImgIn;
    yarp::os::Port  _prtImgOut;
    yarp::os::Port  _configPort;
    HeadEncoderPort _prtHEncsIn;
    yarp::os::BufferedPort<yarp::os::Bottle>  _prtTEncsIn;
    yarp::os::BufferedPort<yarp::os::Bottle>  _prtImuIn;
    ICalibTool *    _calibTool;
    std::string strGroup;

public:

    CamCalibModule();
    ~CamCalibModule();
    
    /** Passes config on to CalibTool */
    virtual bool configure(yarp::os::ResourceFinder &rf);
    virtual bool close();
    virtual bool interruptModule();
    virtual bool updateModule();
    virtual bool respond(const yarp::os::Bottle& command, yarp::os::Bottle& reply);
    virtual double getPeriod();
};


#endif
