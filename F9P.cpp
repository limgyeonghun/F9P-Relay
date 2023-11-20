#include "F9P.hpp"

int main(int argc, char **argv) {
    try {
        string udp_ip = "10.42.0.255";
        int udp_port = 9750;
        string lora_device = "/dev/ttyUSB0";
        string f9p_device = "/dev/ttyACM0";

        MavlinkStream mavlinkStream(udp_ip, udp_port, lora_device);

        F9P f9p(f9p_device, SURVEY_ACC_METER, SURVEY_DURATION_SECS, false, true, &mavlinkStream);
        f9p.run();
    }
    catch (const std::exception &e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1; // Return an error code
    }
    return 0;
}

void F9P::run() {
    if (_f9p) delete _f9p;

    _f9p = new WSerial::Serial();
    _f9p = new WSerial::Serial();
    _f9p->setPortName(_device);
    if (!_f9p->open()) {
        int retries = 60;
        // Give the device some time to come up. In some cases the device is not
        // immediately accessible right after startup for some reason. This can take 10-20s.
        while (retries-- > 0 && _f9p->error() == WSerial::Serial::PermissionError) { // permission error
            cout << "Cannot open device... retrying\n";
            std::this_thread::sleep_for(std::chrono::milliseconds(500));

            if (_f9p->open()) {
                _f9p->clearError();
                break;
            }
        }
        if (_f9p->error() != WSerial::Serial::NoError) {
            cout << "========= Failed to open F9P Device =========\n" << _device << _f9p->errorString();
            throw std::runtime_error("Failed to initialize F9P");
        }
    }
    _f9p->setBaudRate(115200);
//    _f9p->setDataBits(QSerialPort::Data8);
//    _f9p->setParity(QSerialPort::NoParity);
//    _f9p->setStopBits(QSerialPort::OneStop);
//    _f9p->setFlowControl(QSerialPort::NoFlowControl);

    unsigned int baudrate;
    GPSBaseStationSupport *gpsDriver = nullptr;

    while (!_requestStop) {

        if (gpsDriver) {
            delete gpsDriver;
            gpsDriver = nullptr;
        }
        gpsDriver = new GPSDriverUBX(GPSDriverUBX::Interface::UART, &callbackEntry, this, &_reportGpsPos,
                                     _pReportSatInfo);
        baudrate = 0; // auto-configure
        gpsDriver->setSurveyInSpecs(_surveyInAccMeters * 10000.0f, _surveryInDurationSecs);

        if (_useFixedBaseLoction) {
            gpsDriver->setBasePosition(_fixedBaseLatitude, _fixedBaseLongitude, _fixedBaseAltitudeMeters,
                                       _fixedBaseAccuracyMeters * 1000.0f);
        }

        _gpsConfig.output_mode = GPSHelper::OutputMode::RTCM;
        if (gpsDriver->configure(baudrate, _gpsConfig) == 0) {

            /* reset report */
            memset(&_reportGpsPos, 0, sizeof(_reportGpsPos));

            //In rare cases it can happen that we get an error from the driver (eg. checksum failure) due to
            //bus errors or buggy firmware. In this case we want to try multiple times before giving up.
            int numTries = 0;

            while (!_requestStop && numTries < 3) {
                int helperRet = gpsDriver->receive(GPS_RECEIVE_TIMEOUT);

                if (helperRet > 0) {
                    numTries = 0;

                    if (helperRet & 1) {
                        publishGPSPosition();
                        numTries = 0;
                    }

                    if (_pReportSatInfo && (helperRet & 2)) {
                        publishGPSSatellite();
                        numTries = 0;
                    }
                } else {
                    ++numTries;
                }
            }
            if (_f9p->error() != WSerial::Serial::NoError && _f9p->error() != WSerial::Serial::TimeoutError) {
                break;
            }
        }
    }
    cout << "=================== Exiting F9P thread ===================";
}

F9P::F9P(const string &device,
         double surveyInAccMeters,
         int surveryInDurationSecs,
         const std::atomic_bool &requestStop,
         bool enableSatInfo,
         MavlinkStream *mavlinkStream)
        : _mavlinkStream(mavlinkStream), _device(device), _surveyInAccMeters(surveyInAccMeters),
          _surveryInDurationSecs(surveryInDurationSecs), _useFixedBaseLoction(false), _requestStop(requestStop),
          _fixedBaseLatitude(0.0), _fixedBaseLongitude(0.0), _fixedBaseAltitudeMeters(0.0),
          _fixedBaseAccuracyMeters(0.0) {
    cout << "Survey in [accuracy:duration]  -> [" << surveyInAccMeters << " : " << surveryInDurationSecs << "]\n";
    if (enableSatInfo) _pReportSatInfo = new satellite_info_s();
}

F9P::~F9P() {
    if (_pReportSatInfo) delete (_pReportSatInfo);
    if (_f9p) delete (_f9p);
}

void F9P::gotRTCMData(uint8_t *data, size_t len) {
    std::vector<uint8_t> message(data, data + len);
    if (_mavlinkStream) {
//        printf("(%d) %02x %02x %02x %02x %02x %02x %02x %02x\n", len
//               , message[0], message[1], message[2], message[3]
//               , message[4], message[5], message[6], message[7]);
        _mavlinkStream->RTCMDataUpdate(message);
    }
}

int F9P::callbackEntry(GPSCallbackType type, void *data1, int data2, void *user) {
    F9P *gps = (F9P *) user;
    return gps->callback(type, data1, data2);
}

int F9P::callback(GPSCallbackType type, void *data1, int data2) {
    switch (type) {
        case GPSCallbackType::readDeviceData: {
            if (_f9p->bytesAvailable() == 0) {
                int timeout = *((int *) data1);
                if (!_f9p->waitForReadyRead(timeout))
                    return 0; //timeout
            }
            return (int) _f9p->read((char *) data1, data2);
        }
        case GPSCallbackType::writeDeviceData:
            if (_f9p->write((char *) data1, data2) >= 0) {
                if (_f9p->waitForBytesWritten(-1))
                    return data2;
            }
            return -1;

        case GPSCallbackType::setBaudrate:
            return _f9p->setBaudRate(data2) ? 0 : -1;

        case GPSCallbackType::gotRTCMMessage:
            gotRTCMData((uint8_t *) data1, data2);
            break;

        case GPSCallbackType::surveyInStatus: {
            SurveyInStatus *status = (SurveyInStatus *) data1;
//            cout << "Position: " << status->latitude << status->longitude << status->altitude;
            std::cout << "Survey-in duration: " << status->duration
                      << "s cur accuracy: " << std::fixed << std::setprecision(2)
                      << static_cast<float>(status->mean_accuracy) / 1000.0f
                      << "m valid: " << static_cast<int>(status->flags & 1)
                      << " active: " << static_cast<int>((status->flags >> 1) & 1) << "\n";
        }
            break;

        case GPSCallbackType::setClock:
            /* do nothing */
            break;
    }
    return 0;
}

void F9P::publishGPSPosition() {
//    printf("publishGPSPosition\n");
}

void F9P::publishGPSSatellite() {
//    printf("publishGPSSatellite\n");
}
/*
 * RTCM bandwidth: 1.01 kB/s
ret : 42len : 39 Current time: 1695790603834913
ret : 197len : 194 Current time: 1695790604785227
ret : 197len : 194 Current time: 1695790604785302
ret : 197len : 194 Current time: 1695790604785914
ret : 197len : 194 Current time: 1695790604785981
ret : 193len : 190 Current time: 1695790604786252
ret : 185len : 182 Current time: 1695790604787820
ret : 185len : 182 Current time: 1695790604787865
ret : 27len : 24 Current time: 1695790604787890
RTCM bandwidth: 0.53 kB/s
ret : 197len : 194 Current time: 1695790605788374
ret : 197len : 194 Current time: 1695790605788421
ret : 197len : 194 Current time: 1695790605789258
ret : 197len : 194 Current time: 1695790605789313
ret : 193len : 190 Current time: 1695790605789922
ret : 184len : 181 Current time: 1695790605790668
ret : 184len : 181 Current time: 1695790605790717
ret : 27len : 24 Current time: 1695790605790745
ret : 197len : 194 Current time: 1695790606788222
ret : 197len : 194 Current time: 1695790606788291
ret : 197len : 194 Current time: 1695790606789303
ret : 197len : 194 Current time: 1695790606789378
RTCM bandwidth: 1.21 kB/s
ret : 193len : 190 Current time: 1695790606790131
ret : 175len : 172 Current time: 1695790606790862
ret : 175len : 172 Current time: 1695790606790930
ret : 27len : 24 Current time: 1695790606790969
ret : 197len : 194 Current time: 1695790607787849
ret : 197len : 194 Current time: 1695790607787911
ret : 197len : 194 Current time: 1695790607789049
ret : 197len : 194 Current time: 1695790607789264
ret : 193len : 190 Current time: 1695790607789311
ret : 175len : 172 Current time: 1695790607789917
ret : 175len : 172 Current time: 1695790607789965
 */