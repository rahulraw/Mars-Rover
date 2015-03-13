/*
 *  Player - One Hell of a Robot Server
 *  Copyright (C) 2008-2010  Willow Garage
 *                      
 *
 *  This library is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 2.1 of the License, or (at your option) any later version.
 *
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this library; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */


#ifndef MS_3DMGX3_HH
#define MS_3DMGX3_HH

#include <fstream>
#include <stdexcept>
#include <stdint.h>
#include <vector>


namespace microstrain_3dmgx3_imu
{

  struct MIP_Data_Response {
    unsigned char desc_set;
    unsigned char command_desc;
    unsigned char command_echo;
    unsigned char error_code;
    double x,  y, z;
    float position_acc;
    unsigned short flags;
    unsigned int time;
    double ftime;
    float orientation[9];
  };

  //! Macro for defining exception (std::runtime_error should be top parent)
  #define DEF_EXCEPTION(name, parent) \
  class name  : public parent { \
  public: \
    name(const char* msg) : parent(msg) {} \
  }

  DEF_EXCEPTION(Exception, std::runtime_error);
  DEF_EXCEPTION(TimeoutException, Exception);
  DEF_EXCEPTION(CorruptedDataException, Exception);

  #undef DEF_EXCEPTION

  //! A class for interfacing to the microstrain 3dmgx3 and inertialink IMUs
  /*!
   * Note: This class is unreviewed and unsupported. It may change at any
   * time without notice.
   *
   * Many of the methods within this class may throw an
   * microstrain_3dmgx3_imu::exception, timeout_exception, or
   * corrupted_data_exception.
   *
   * Before using the IMU, it must be opened via the open_port method.
   * When finished using, it should be closed via the close_port
   * method.  Alternatively, close port will get called at
   * destruction.
   *
   * The library is primarily designed to be used in continuous mode,
   * which is enabled with the set_continuous method, and then serviced
   * with one of the receive methods.
   *
   * Implementation of specific polled message transactions can be
   * done with the transact method.
   *
   * Because the timing related to the USB stack is fairly
   * non-deterministic, but the IMU is internally known to be clocked
   * to a 100hz clock, we have wrapped a Kalman filter around calls to
   * get system time, and the internal imu time.  This is only known
   * to be reliable when operating in continuous mode, and if init_time
   * is called shortly prior to beginning to get readings.
   * 
   *
   * Example code:
   * \code
   *   microstrain_3dmgx3_imu::IMU imu;
   *   imu.open_port("/dev/ttyUSB0");
   *   imu.init_time();
   *   imu.init_gyros();
   *   imu.set_continuous(microstrain_3dmgx3_imu::IMU::CMD_ACCEL_ANGRATE_ORIENT);
   *   while (int i = 0 ; i < 100; i++)
   *   {
   *     double accel[3];
   *     double angrate[3];
   *     double orientation[9];
   *     imu.receive_accel_angrate_orientation(&time, accel, angrate, orientation);
   *   }
   *   imu.close_port();
   * \endcode
   */
  class IMU
  {
    //! IMU internal ticks/second
    //! Maximum bytes allowed to be skipped when seeking a message
    static const int MAX_BYTES_SKIPPED  = 1000;
    //! Number of KF samples to sum over
    static const unsigned int KF_NUM_SUM= 100;
    //! First KF term
    static const double KF_K_1          = 0.00995031;
    //! Second KF term
    static const double KF_K_2          = 0.0000497506;

  public: 

    //! Gravity (m/sec^2)
    static const double G               = 9.80665;    

    //! Constructor
    IMU();

    // Destructor
    ~IMU();

    //! Open the port
    /*! 
     * This must be done before the imu can be used.
     * 
     * \param port_name   A character array containing the name of the port
     *
     */
    void openPort(const char *port_name);

    //! Close the port
    void closePort();

    //! Initialize timing variables.
    /*!
     * This call determines the initial offset of the imu relative to 
     * system clock time, and resets the kalman filter state.
     *
     * \param fix_off this fixed offset will be added to the timestamp of the imu
     */
    void initTime();

    //! Initial gyros
    /*! 
     * This call will prompt the IMU to run its gyro initialization
     * routine.  
     *
     * NOTE: THE IMU MUST BE STATIONARY WHEN THIS ROUTINE IS CALLED
     *
     * \param bias_x   Pointer to double where x bias will be placed.
     * \param bias_y   Pointer to double where y bias will be placed.
     * \param bias_z   Pointer to double where z bias will be placed.
     */
    void initGyros(double* bias_x = 0, double* bias_y = 0, double* bias_z = 0);

    //! Put the device in continuous mode
    /*!
     * This call puts the IMU into a mode where it is continuously
     * outputting a particular message.
     *
     * \param command   The type of message to be output.
     * 
     * \return  Whether or not continuous mode was enabled successfully.
     */
    bool setContinuous();
    
    //! Take the device out of continous mode.
    void stopContinuous();

    //! Read a message of type "ACCEL_ANGRATE"
    /*! 
     * \param time    Pointer to uint64_t which will receive time
     * \param accel   array of accelerations which will be filled
     * \param angrate array of angular rates which will be filled
     */
    void receiveAccelAngrate(uint64_t *time, double accel[3], double angrate[3]);

    void receiveAccelAngrateOrientation(uint64_t *time, double accel[3], double angrate[3], double orientation[9]);

    void setFixedOffset(double fix_off) {fixed_offset = fix_off;};

  private:
    bool checkAckResponse(const MIP_Data_Response& resp, unsigned char cmd);
    void checkDataResponse(const MIP_Data_Response& resp);
    bool send_command(const char command_string[]);

    int readPort(std::vector<MIP_Data_Response>& resp);

    //! Extract time from a pointer into an imu buffer
    uint64_t extractTime(unsigned int time);

    //! Run the filter on the imu time and system times
    double filterTime(double imu_time, double sys_time);


    //! The file descriptor
    int fd;

    //! The number of times the imu has wrapped
    uint32_t wraps;

    //! The number of ticks the initial offset is off by
    uint32_t offset_ticks;

    //! The last number of ticks for computing wraparound
    uint32_t last_ticks;

    //! The different in the number of ticks
    uint32_t diff_ticks;

    //! The time at which the imu was started
    unsigned long long start_time;

    //! The estimate of time offset and driftrate
    double time_est[2];

    //! The covariances on time offset and driftrate
    double P_time_est[2][2];

    //! Whether continuous mode is enabled
    bool continuous;

    //! A counter used by the filter
    unsigned int counter;

    //! Variables used by the kalman computation
    double fixed_offset, offset, d_offset, sum_meas;

  };

}
#endif
