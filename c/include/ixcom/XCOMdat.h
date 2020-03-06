#ifndef XCOMDAT_H_
#define XCOMDAT_H_

#include <stdint.h>

#define NMEA_MAX_COM 6     /* TODO: will be removed */
#define NMEA_MAX_MSG_COM 8 /* TODO: will be removed */
#define NMEA_MAX_UDP 6     /* TODO: will be removed */
#define NMEA_MAX_MSG_UDP 8 /* TODO: will be removed */

/*
 * General
 */
#define XCOM_MAX_CHANNELS 32 /**< Number of independent iXCOM channels */
#define XCOM_MAX_MESSAGE_LENGTH 4096 /**< Max. length of iXCOM frames */
#define XCOM_MAX_NUMBER_OF_TEMPERATURES                                        \
  16 /**< Max. number of supported system temperatures */
#define XCOM_MAX_NUMBER_OF_ARINC825_FRAMES                                     \
  29 /**< Max. number of supported ARINC825 frames */
#define XCOM_MAX_CLIENT_SUPPORT                                                \
  8 /**< Max. number of supported clients for MVT feature */
#define XCOM_MAX_CLIENT_LOGS                                                   \
  8 /**< Max. number of supported logs per client (MVT) */
#define XCOM_MAX_SATCHANNELS                                                   \
  128                        /**< Max. number of supported satellite channels */
#define XCOM_INTERFACE_LAN 0 /**< Needed for PARXCOM_NETCONFIG */
#define XCOM_INTERFACE_USB 1 /**< Needed for PARXCOM_NETCONFIG */
#define XCOM_SYNC_BYTE 0x7E  /**< Synchronization character */
/*
 * iXCOM Message IDs
 */
enum XComMessageID {

  XCOM_MSGID_IMURAW = 0x00,  /**< IMU data - calibrated */
  XCOM_MSGID_IMUCORR = 0x01, /**< IMU data - calibrated and corrected for errors
                                estimated by EKF based data fusion */
  XCOM_MSGID_IMUCOMP =
      0x02, /**< IMU data - calibrated and corrected for estimated errors,
               compensated for earth rate and gravity */
  XCOM_MSGID_IMUCAL = 0x31,     /**< IMU special data, used for calibration */
  XCOM_MSGID_OMGINT = 0x35,     /**< Integrated angular rate */
  XCOM_MSGID_OMGDOT = 0x50,     /**< Angular acceleration around body axes */
  XCOM_MSGID_LOADFACTOR = 0x49, /**< Load factor */
  XCOM_MSGID_TEMPERATURE =
      0x22, /**< IMU dependent system temperatures (CPU, PCB and sensors) */
  XCOM_MSGID_POWER =
      0x21, /**< Internal voltage measurements (IMU, FPGA, ISO, VMS, ...) */
  XCOM_MSGID_TIME =
      0x26, /**< Time related data, regarding system, PPS, IMU and GNSS */
  XCOM_MSGID_EVENTTIME =
      0x34, /**< Time measurement of the external event inputs */
  XCOM_MSGID_IMUFILTERED =
      0x56, /**< IMU data with butterworth filtering (@deprecated) */

  XCOM_MSGID_INSSOL = 0x03, /**< Integrated navigation solution including IMU
                               data in a selectable frame */
  XCOM_MSGID_INSSOLECEF = 0x47, /**< Integrated navigation solution including
                                   IMU data in ECEF frame */
  XCOM_MSGID_INSRPY =
      0x04, /**< Integrated attitude solution (roll, pitch, yaw) */
  XCOM_MSGID_INSDCM =
      0x05, /**< Integrated attitude solution (direction cosine matrix) */
  XCOM_MSGID_INSQUAT = 0x06,   /**< Integrated attitude solution (quaternion) */
  XCOM_MSGID_INSVELNED = 0x07, /**< Integrated velocity solution in NED frame */
  XCOM_MSGID_INSVELENU = 0x23, /**< Integrated velocity solution in ENU frame */
  XCOM_MSGID_INSVELECEF =
      0x08, /**< Integrated velocity solution in ECEF frame */
  XCOM_MSGID_INSVELBODY =
      0x09, /**< Integrated velocity solution in body frame */
  XCOM_MSGID_INSPOSLLH = 0x0A,  /**< INS/GNSS position in ECEF (WGS84) frame */
  XCOM_MSGID_INSPOSECEF = 0x0B, /**< INS/GNSS position in UTM frame */
  XCOM_MSGID_INSPOSUTM = 0x0C,  /**< INS/GNSS position in UTM frame */
  XCOM_MSGID_INSROTTEST = 0x0D, /**< INS rotation test */
  XCOM_MSGID_INSGNDSPEED =
      0x13, /**< INS speed over ground, course over ground and down velocity */
  XCOM_MSGID_INSTRACKACC =
      0x0E, /**< INS along-track, cross-track and vertical acceleration */
  XCOM_MSGID_INSMAGHDG =
      0x1A, /**< INS true heading converted to magnetic heading */
  XCOM_MSGID_INSMGRS =
      0x46, /**< INS/GNSS position solution in Military Grid Reference System */

  XCOM_MSGID_EKFSTDDEV = 0x0F, /**< EKF estimated standard deviations */
  XCOM_MSGID_EKFSTDDEV2 =
      0x28, /**< EKF estimated standard deviations (@deprecated; use EKFSTDDEV2
               for new developments) */
  XCOM_MSGID_EKFERROR = 0x10, /**< EKF estimated sensor errors (@deprecated; use
                                 EKFERROR2 for new developments) */
  XCOM_MSGID_EKFERROR2 = 0x27,   /**< EKF estimated sensor errors */
  XCOM_MSGID_EKFPOSCOVAR = 0x29, /**< EKF position covariance matrix */
  XCOM_MSGID_EKFTIGHTLY =
      0x11, /**< EKF used satellite information in tightly coupled mode */
  XCOM_MSGID_EKFSTDDEVECEF =
      0x48, /**< EKF estimated standard deviations in ECEF frame */

  XCOM_MSGID_GNSSSOL =
      0x12, /**< GNSS navigation solution. This package contains GNSS position,
               velocity, number of satellites, PDOP and receiver status. */
  XCOM_MSGID_GNSSTIME = 0x14,    /**< GNSS time stamp (UTC) */
  XCOM_MSGID_GNSSSOLCUST = 0x15, /**< GNSS custom solution */
  XCOM_MSGID_GNSSSATINFO = 0x25, /**< GNSS satellite status information */
  XCOM_MSGID_GNSSHDG = 0x33, /**< GNSS attitude information (only available in
                                dual-antenna systems) */
  XCOM_MSGID_GNSSLEVERARM = 0x1B, /**< GNSS lever arm estimates */
  XCOM_MSGID_GNSSVOTER = 0x1C,    /**< GNSS voter information */
  XCOM_MSGID_GNSSALIGNBSL = 0x38, /**< GNSS RTK quality ENU baselines */
  XCOM_MSGID_GNSSSATINF = 0x51,   /**< GNSS satellite status information */
  XCOM_MSGID_GNSSSTREAM =
      0x58, /**< GNSS raw data stream in proprietary binary protocol (depends on
               internal receiver type) */
  XCOM_MSGID_GNSSOBSSTREAM =
      0x59, /**< GNSS correction data in proprietary binary protocol (depends on
               the selected data format) */
  XCOM_MSGID_GNSSDOP =
      0x60, /**< DOP values for the satellites used in the GNSS solution */
  XCOM_MSGID_GNSSHWMON =
      0x1E, /**< GNSS hardware monitor (voltages and temperatures of the
               integrated GNSS receiver) */
  XCOM_MSGID_PORTSTATS =
      0x43, /**< COM port statistics of the integrated GNSS receiver */
  XCOM_MSGID_GNSSPOSECEF =
      0x61, /**< GNSS position solution in ECEF coordinates */
  XCOM_MSGID_GNSSVELECEF =
      0x62, /**< GNSS velocity solution in ECEF coordinates */

  XCOM_MSGID_PASSTHROUGH = 0x63,

  XCOM_MSGID_HEAVE = 0x1F,     /**< Heave information (@deprecated) */
  XCOM_MSGID_WHEELDATA = 0x16, /**< (Single-) Odometer measurements */
  XCOM_MSGID_AIRDATA = 0x17,   /**< Air data computer measurements */
  XCOM_MSGID_MAGDATA = 0x18,   /**< Magnetometer measurements */
  XCOM_MSGID_ADC24DATA =
      0x37, /**< ADC measurements of the optional ADC24 board */
  XCOM_MSGID_CSACDATA =
      0x42, /**< Status information of the optional chip scale atomic clock */
  XCOM_MSGID_STEPDETECTION =
      0x71, /**< Status information if the integrated step detection FSM */

  XCOM_MSGID_SYSSTAT = 0x19,  /**< Extended system status */
  XCOM_MSGID_CANSTAT = 0x24,  /**< CAN controller status information */
  XCOM_MSGID_STATFPGA = 0x20, /**< FPGA status information */
  XCOM_MSGID_ADC24STATUS =
      0x36, /**< Status information of the optional ADC24 board */
  XCOM_MSGID_ARINC429STAT = 0x1D, /**< ARINC429 status information */
  XCOM_MSGID_MONITOR =
      0x57, /**< Monitor messages (for debugging purpose only) */

  XCOM_MSGID_MVCSLAVE =
      0x39, /**< Slave measurements (multi-vehicle communication module) */
  XCOM_MSGID_MVCDATA =
      0x41, /**< Master measurements (multi-vehicle communication module) */

  XCOM_MSGID_POSTPROC = 0x40, /**< Log containing online navigation solution as
                                 well as data needed for postprocessing */

  XCOM_MSGID_NMEA0183_GGA =
      0x72, /**< NMEA-0183: Time, position, and fix related data */
  XCOM_MSGID_NMEA0183_GLL =
      0x73, /**< NMEA-0183: Geographic Position - Latitude/Longitude */
  XCOM_MSGID_NMEA0183_GSA =
      0x74, /**< NMEA-0183: GNSS DOP and active satellites */
  XCOM_MSGID_NMEA0183_HDT = 0x75, /**< NMEA-0183: True heading */
  XCOM_MSGID_NMEA0183_RMC =
      0x76, /**< NMEA-0183: Recommended Minimum Navigation Message */
  XCOM_MSGID_NMEA0183_VTG =
      0x77, /**< NMEA-0183: Track Made Good and Ground Speed Message */
  XCOM_MSGID_NMEA0183_ZDA = 0x78, /**< NMEA-0183: Time and Date - UTC, Day,
                                     Month, Year and Local Time Zone Message */
  XCOM_MSGID_NMEA0183_GST =
      0x79, /**< NMEA-0183: GNSS Pseudorange Error Statistics Message */
  XCOM_MSGID_NMEA0183_PIAHS =
      0x7A, /**< NMEA-0183: Attitude, Heading, Heave and Standard Deviation
               Message (iMAR Proprietary) */
  XCOM_MSGID_NMEA0183_PISTATUS1 =
      0x7B, /**< NMEA-0183: Status Information Message (iMAR Proprietary) */

  /* -------------------------------------- */
  XCOM_USERIF_LATENCYRX =
      0xF1, /**< @deprecated; will be removed in the near future */
  XCOM_USERIF_LATENCYTX =
      0xF2, /**< @deprecated; will be removed in the near future */
  XCOM_USERIF_ABDDUMMY =
      0xF3, /**< @deprecated; will be removed in the near future */
  XCOM_USERIF_INSSOL_SCU_NED2 =
      0xF4, /**< @deprecated; will be removed in the near future */
  XCOM_USERIF_INSSOL_SCU_NED =
      0xF5, /**< @deprecated; will be removed in the near future */
  XCOM_USERIF_INSSOL_SCU_ECEF = 0xF6, /**< Integrated navigation solution and
                                         status information for iSCU */
  XCOM_USERIF_INSSOL =
      0xF7, /**< @deprecated; will be removed in the near future */
  XCOM_USERIF_THALES =
      0xF8, /**< @Deprecated; will be removed in the near future */
  XCOM_USERIF_THALESLS =
      0xF9, /**< @deprecated; will be removed in the near future */
  XCOM_USERIF_THALESCOV =
      0xFA, /**< @deprecated; will be removed in the near future */
  XCOM_USERIF_THALESAIR =
      0xFB, /**< @deprecated; will be removed in the near future */
  XCOM_USERIF_ADSE = 0xFC,    /**< Integrated navigation solution and status
                                 information for ADSE framework */
                              /* -------------------------------------- */
  XCOM_MSGID_COMMAND = 0xFD,  /**< iXCOM command ID */
  XCOM_MSGID_RESPONSE = 0xFE, /**< iXCOM response ID */
  XCOM_MSGID_PARAMETER = 0xFF /**< iXCOM parameter ID */
};

/**
 * iXCOM COMMAND IDs:
 * ------------------
 * The iXCOM protocol allows sending commands via the command message ID 0xFD
 * (XCOM_MSGID_COMMAND), The command message payload consists of a header
 * selecting a specific command and a variable payload.
 */
enum XCOMcmdCategory {
  XCOM_CMDID_LOG = 0x0000, /**< The LOG command enables/disables the output of
                              several packages. */
  XCOM_CMDID_EXT =
      0x0001, /**< @deprecated; will be removed in the near future */
  XCOM_CMDID_CAL =
      0x0002, /**< @deprecated; will be removed in the near future */
  XCOM_CMDID_CONF = 0x0003, /**< This set of commands saves and loads a
                               configuration to/from internal storage */
  XCOM_CMDID_EKF = 0x0004, /**< Set of commands to save current position/heading
                              and to start and alignment */
  XCOM_CMDID_XCOM = 0x0005, /**< Set of commands to control the behavior of the
                               XCOM protocol module */
  XCOM_CMDID_FPGA = 0x0006, /**< Set of commands to control the FPGA module */
  XCOM_CMDID_EXTAID =
      0x0007, /**< Set of commands to control the external aiding sources */
  XCOM_CMDID_PATH_CTRL =
      0x0008, /**< Set of commands to control the path following module */
  XCOM_CMDID_USR_IF =
      0x0009, /**< Set of commands to control application specific modules */
  XCOM_CMDID_DUMP = 0x000A, /**< Set of commands to control the behavior of the
                               dumper module */

  XCOM_CMDID_MAX = XCOM_CMDID_DUMP
};

/**
 * iXCOM XCOM command:
 * -------------------
 * The iXCOM commands must be used to initialize/open or close the iXCOM data
 * interface. The iXCOM command needs as parameter the corresponding channel
 * number.
 */
enum XCOMcmd_XCom {
  XCOM_CMDXCOM_CLOSE = 0x0000,     /**< Close an iXCOM channel */
  XCOM_CMDXCOM_OPEN = 0x0001,      /**< Open an iXCOM channel */
  XCOM_CMDXCOM_COLDRESET = 0x0002, /**< Performing a cold reset (complete
                                      removal of power and restart) */
  XCOM_CMDXCOM_WARMRESET =
      0x0003, /**< Performing a warm reset (forcing a restart via a reboot with
                 out powering down) */
  XCOM_CMDXCOM_RESETOMGINT = 0x0004,   /**< Reset OMG integration */
  XCOM_CMDXCOM_UPDATESVN = 0x0005,     /**< Load baseline via SVN */
  XCOM_CMDXCOM_RESETTIMEBIAS = 0x0006, /**< Reset estimated time bias */
  XCOM_CMDXCOM_SKIPLOGFILE = 0x0007,   /**< Skip log-file (close the current
                                          log-file and open a new one) */
  XCOM_CMDXCOM_TRACELOG =
      0x0008, /**< Start trace logger (for debugging purpose only) */
  XCOM_CMDXCOM_BACKUPAPP = 0x0009, /**< Creating a backup image of the active
                                      application partition */
  XCOM_CMDXCOM_BACKUPCALIB =
      0x000A, /**< Creating a backup image of the calibration partition */
  XCOM_CMDXCOM_BACKUPROOT =
      0x000B, /**< Creating a backup image of the active root partition */
  XCOM_CMDXCOM_BACKUPIFS =
      0x000C, /**< Creating a backup image of the active ifs partition */
  XCOM_CMDXCOM_BACKUPSYSTEM = 0x000D, /**< Creating a eMMC backup image */
  XCOM_CMDXCOM_UPDATEAPP =
      0x000E, /**< Updating the application partition (the image must be stored
                 as </data/upload/appfs_qnx6fs.imar>) */
  XCOM_CMDXCOM_UPDATECALIB =
      0x000F, /**< Updating the calibration partition (the image must be stored
                 as </data/upload/calibfs_qnx6fs.imar>) */
  XCOM_CMDXCOM_UPDATEROOT =
      0x0010, /**< Updating the root partition (the image must be stored as
                 </data/upload/rootfs_qnx6fs.imar>) */
  XCOM_CMDXCOM_UPDATEIFS =
      0x0011, /**< Updating the ifs partition (the image must be stored as
                 </data/upload/ifs_fat.imar>) */

  XCOM_CMDXCOM_MAX = XCOM_CMDXCOM_UPDATEIFS
};

/**
 * iXCOM LOG command:
 * ------------------
 * The LOG command manipulates the log list of the currently active channel.
 * Besides a specific message ID, a Trigger, a Command Parameter and a Divider
 * value are expected.
 */
enum XComLogCmdParam {
  XCOM_CMDLOG_ADD = 0x0000,  /**< This parameter adds a log to the log-list of
                                the IMS on the currently used Channel Number. */
  XCOM_CMDLOG_STOP = 0x0001, /**< Stop data output of a specific message. The
                                selected log is still contained in the log list
                                of the IMS, but output will be interrupted. */
  XCOM_CMDLOG_START =
      0x0002, /**< Start data output of a previously stopped log. */
  XCOM_CMDLOG_CLEAR = 0x0003, /**< Delete a specific log from the log-list. */
  XCOM_CMDLOG_CLEARALL =
      0x0004, /**< This command clears the log-list of the current channel. */
  XCOM_CMDLOG_STOPALL =
      0x0005, /**< Stop data output on this channel. The configured data logs
                 are still contained in the log-list, but output will be
                 interrupted. */
  XCOM_CMDLOG_STARTALL = 0x0006, /**< Start data output of all configured data
                                    logs of the current log list. */

  XCOM_CMDLOG_MAX = XCOM_CMDLOG_STARTALL
};

/**
 * iXCOM LOG command trigger type:
 * -------------------------------
 * To every log, a trigger to generate a message has to be chosen out of the
 * following possibilities.
 */
enum XComLogTrigger {
  XCOM_CMDLOG_TRIG_SYNC = 0x00, /**< The log is generated synchronously with the
                                   IMU sample clock. */
  XCOM_CMDLOG_TRIG_EVENT =
      0x01, /**< This log if an event on . The message will be generated and
               sent immediately (asynchronously to the IMU sample clock) as soon
               as new data are available (related to the requested log). */
  XCOM_CMDLOG_TRIG_POLLED =
      0x02, /**< This message log will be generated once immediately after a re-
               quest is received. */
  XCOM_CMDLOG_TRIG_EXTEVENT =
      0x03, /**< Log generation is triggered via input by an external event. */
  XCOM_CMDLOG_TRIG_PPTEVENT =
      0x04, /**< Log generation is triggered via PPT event. */
  XCOM_CMDLOG_TRIG_AVG =
      0x05, /**< The log is averaged over a specific time period */
  XCOM_CMDLOG_TRIG_PPS =
      0x06, /**< Log generation is triggered via PPS event. */

  XCOM_CMDLOG_TRIG_MAX = XCOM_CMDLOG_TRIG_PPS
};

/**
 * iXCOM CONF command:
 * -------------------
 * The CONF command can be used to store or load the system configuration to or
 * from internal storage.
 */
enum XCOMcmd_Conf {
  XCOM_CMDCONF_SAVE =
      0x0000, /**< Save the current configuration to the internal storage of the
                 system. This configuration will be loaded during the next
                 system boot. */
  XCOM_CMDCONF_LOAD = 0x0001, /**< Load the stored configuration into RAM. */
  XCOM_CMDCONF_DELIVERYLOAD =
      0x0002, /**< The current configuration will be deleted and the delivery
                 setting will be restored after an automatic reboot */
  XCOM_CMDCONF_DELIVERYSAVE = 0x0003, /**< Saves current configuration files and
                                         stores them delivery configuration */
  XCOM_CMDCONF_FACTORYLOAD =
      0x0004, /**< The current configuration will be deleted and the factory
                 setting will be restored after an automatic reboot  */

  XCOM_CMDCONF_MAX = XCOM_CMDCONF_FACTORYLOAD
};

/**
 * iXCOM EKF command:
 * -----------------
 */
enum XCOMcmd_Ekf {
  XCOM_CMDEKF_STARTALIGN = 0x0000, /**< Starting an alignment with the existing
                                      alignment parameters */
  XCOM_CMDEKF_STOREPOS =
      0x0001, /**< The INS will store the current position to the internal
                 storage. The stored position will be loaded when the STOREDPOS
                 mode is selected in the PAREKF_STARTUPV2 parameter. */
  XCOM_CMDEKF_STOREHDG =
      0x0002, /**< The INS will store the current heading to the internal
                 storage. The stored heading will be loaded when the STOREDHDG
                 mode is selected in the PAREKF_STARTUPV2 parameter. */
  XCOM_CMDEKF_STOREANTOFFSET =
      0x0003, /**< Store the estimated antenna offset to the configuration
                 loaded in RAM. To make the estimated antenna offset persist
                 after a reboot, the CONF Command has to be issued with the SAVE
                 parameter. */
  XCOM_CMDEKF_FORCEDZUPT = 0x0004, /**< @deprecated */
  XCOM_CMDEKF_ALIGNCOMPLETE =
      0x0005, /**< If this command is received the INS will complete the static
                 alignment. Motion is allowed after issuing this command */
  XCOM_CMDEKF_COPYODOSCF =
      0x0006, /**< Save estimated odometer scale factor to the configuration
                 loaded in RAM. To make this setting persist after a reboot, the
                 CONF Command has to be issued with the SAVE parameter. */
  XCOM_CMDEKF_LOADPARFROMFILE = 0x0007, /**< Load EKF specific default parameter
                                           from read-only configuration file */
  XCOM_CMDEKF_STARTZUPTCALIBRATION =
      0x0008, /**< If the EKF is initialized successfully, start Zero Velocity
                 Update calibration */
  XCOM_CMDEKF_STOREMAGSTATE =
      0x0009, /**< If the stationary alignment has completed successfully, start
                 the iMAG calibration. */
  XCOM_CMDEKF_DELETEMAGSTATE =
      0x000A, /**< Remove iMAG calibration files from system. */

  XCOM_CMDEKF_MAX = XCOM_CMDEKF_DELETEMAGSTATE
};

/**
 * iXCOM FPGA command:
 * -------------------
 * This command is not intended to be used by the common operator! Wrong usage
 * may damage or destroy the system
 *
 * NOTE:
 * Before usage contact iMAR design engineer team for approval
 * (support@imar-navigation.de).
 */
enum XCOMcmd_Fpga {
  XCOM_CMDFPGA_IMUPOWER =
      0x0000, /**< @deprecated: Use XCOMPAR_PARFPGA_POWER instead */
  XCOM_CMDFPGA_GNSSPOWER =
      0x0001, /**< @deprecated: Use XCOMPAR_PARFPGA_POWER instead */
  XCOM_CMDFPGA_ISOPOWER =
      0x0002, /**< @deprecated: Use XCOMPAR_PARFPGA_POWER instead */
  XCOM_CMDFPGA_VMSPOWER =
      0x0003, /**< @deprecated: Use XCOMPAR_PARFPGA_POWER instead */
  XCOM_CMDFPGA_SHUTDOWN =
      0x0004, /**< Leaving operational mode and entering standby mode. If this
                 command is received, all internal components will powered down.
               */
  XCOM_CMDFPGA_POWERUP =
      0x0005, /**< Leaving standby mode and entering operational mode. If this
                 command is received, the system will perform an initial
                 alignment and starts navigation. All internal components will
                 be powered up*/
  XCOM_CMDFPGA_FLUSHFRAM = 0x0006, /**< If the command is received, the system
                                      will flush data to the FRAM if possible */

  XCOM_CMDFPGA_MAX = XCOM_CMDFPGA_FLUSHFRAM
};

/**
 * iXCOM EXTAID command:
 * ---------------------
 * The EXTAID command controls the external aiding sources inside the INS.
 * This command can be used to send measurement updates directly to the EKF
 * module. In addition to that forced aiding commands are also supported. The
 * forced aiding will be applied inside the filter until the aiding switch will
 * be set to 0
 */
enum XCOMcmd_Extaid {
  XCOM_CMDEXTAID_FREEZE_ALT = 0x0000, /**< Freeze altitude output */
  XCOM_CMDEXTAID_FREEZE_HDG = 0x0001, /**< Freeze heading output */
  XCOM_CMDEXTAID_FREEZE_VEL = 0x0002, /**< Freeze velocity ourout */
  XCOM_CMDEXTAID_POS_WGS84 = 0x0003,  /**< External position aiding in WGS84 */
  XCOM_CMDEXTAID_VEL_NED =
      0x0004,                  /**< External velocity aiding in NED frame. */
  XCOM_CMDEXTAID_HDG = 0x0005, /**< External heading aiding. */
  XCOM_CMDEXTAID_HGT = 0x0006, /**< External height aiding.*/
  XCOM_CMDEXTAID_VEL_BODY =
      0x0007, /**< External velocity aiding in INS body frame. */

  XCOM_CMDEXTAID_MAX = XCOM_CMDEXTAID_HGT
};

/**
 * XCOM Parameter
 * -----------------------------
 *
 * The iXCOM protocol provides the ability to set or request all system
 * parameters separately. The parameters can be set via the binary interface.
 * The binary interface communicates via the parameter channel ID 0xFF
 * ('XCOM_MSGID_PARAMETER').
 */
enum XCOMpar_action {
  XCOM_PAR_SET = 0x00, /**< Changing a XCOM parameter */
  XCOM_PAR_GET = 0x01  /**< Reading a XCOM parameter */
};

enum XComParameterID {

  /* SYS */
  XCOMPAR_PARSYS_PRJNUM =
      0, /**< This parameter is read-only and will be factory set during INS
            production. It contains the iMAR project number under which the
            system has been manufactured. */
  XCOMPAR_PARSYS_PARTNUM =
      1, /**< This parameter is read-only and will be factory set during
            production. It contains the part number (article number) including
            revision letter of the INS. */
  XCOMPAR_PARSYS_SERIALNUM =
      2, /**< This parameter is read-only and will be factory set during
            production. It contains the serial number of the INS. */
  XCOMPAR_PARSYS_MFG =
      3, /**< This parameter is read-only and will be factory set during
            production. It contains the manufacturing date of the INS. */
  XCOMPAR_PARSYS_CALDATE =
      4, /**< This parameter is read-only and will be factory set during
            calibration. It contains the last calibration date of the INS. */
  XCOMPAR_PARSYS_FWVERSION =
      5, /**< This parameter is read-only and will be factory set during
            production. It contains the firmware version currently operating on
            the INS. */
  XCOMPAR_PARSYS_NAVLIB =
      6, /**< This parameter is read-only and will be factory set during
            production. It contains the NAVLIB version currently installed on
            the INS. */
  XCOMPAR_PARSYS_EKFLIB =
      7, /**< This parameter is read-only and will be factory set during
            production. It contains the version number of the EKFLIB currently
            installed on the device. */
  XCOMPAR_PARSYS_EKFPARSET =
      8, /**< This parameter is read-only and will be factory set during
            production. It contains the name of the EKF parameter set being used
            currently. */
  XCOMPAR_PARSYS_NAVNUM =
      9, /**< This parameter is read-only and will be factory set during
            production. It contains the version number of the strapdown
            navigation module installed on the system. */
  XCOMPAR_PARSYS_NAVPARSET = 10, /**< This parameter is read-only and will be
                                    factory set during production. */
  XCOMPAR_PARSYS_MAINTIMING =
      11, /**< This parameter is read-only and will be factory set during
             production. This parameter contains the nominal sampling rate in Hz
             of the integrated inertial sensors. Writing of this parameter is
             password-protected. */
  XCOMPAR_PARSYS_PRESCALER =
      12, /**< This parameter is read-only and will be factory set during
             production. It contains the downsampling factor which is used in
             the internal compensation methods, such as the coning and sculling
             correction module. The prescaler also affects the maximum rate
             which is applicable through the message log mechanism. Writing this
             parameter is password-protected. */
  XCOMPAR_PARSYS_UPTIME =
      13, /**< This parameter is read-only. The parameter contains the system’s
             current uptime (time since last reboot) in [sec]. */
  XCOMPAR_PARSYS_OPHOURCNT =
      14, /**< This parameter is read-only. It contains the accumulated
             operating time in [sec] over the lifetime of the system. */
  XCOMPAR_PARSYS_BOOTMODE =
      15, /**< This parameter defines the boot behavior of the system. In the
             default setting (= 0), the system will start in normal operational
             mode. When setting this parameter to standby (= 1), the system will
             boot into power saving mode and to enter normal operational mode,
             the wakeup command will have to be sent. */
  XCOMPAR_PARSYS_FPGAVER =
      16, /**< This read-only parameter contains the FPGA firmware version. */
  XCOMPAR_PARSYS_CONFIGCRC =
      17, /**< This read-only parameter holds the checksums calculated over the
             internal configuration binary files. */
  XCOMPAR_PARSYS_OSVERSION =
      18, /**< This read-only parameter contains the version number of the host
             operating system. */
  XCOMPAR_PARSYS_SYSNAME = 19, /**< This read-only parameter contains the
                                  product name, e.g. iNAT-FSSG-01. */
  XCOMPAR_PARSYS_MINFPGAVER =
      20, /**< This read-only parameter defines the minimum required FPGA
             firmware version for the firmware installed on the device. If the
             actual FPGA version is lower than the version given in this
             parameter, some firmware features may not be available. */
  XCOMPAR_PARSYS_SYNCMODE =
      21, /**< This parameter defines the time synchronization source. */
  XCOMPAR_PARSYS_CALIBID =
      22, /**< This parameter defines the calibration ID which is need fpr an
             unique identifier during the factory calibration */
  XCOMPAR_PARSYS_PTP = 23, /* ToDO @ JoKa */
  XCOMPAR_PARSYS_EEPROM =
      24, /**< This parameter holds the content of the EEPROM device. The EEPROM
             device stores hardware dependent information */

  /* IMU */
  XCOMPAR_PARIMU_MISALIGN =
      105, /**< This parameter sets the installation misalignment between the
              INS enclosure and the vehicle frame. */
  XCOMPAR_PARIMU_TYPE =
      107, /**< This parameter defines the inertial sensor type and will be
              factory set during production. */
  XCOMPAR_PARIMU_LATENCY =
      108, /**< This parameter adjusts the timestamp of the inertial data which
              is used to synchronize IMU measurements with external sensors */
  XCOMPAR_PARIMU_CALIBCOARSE =
      109, /**< This parameter defines calibration parameters which will be used
              by the system to enable additional correction of the calibrated
              IMU values. */
  XCOMPAR_PARIMU_CROSSCOUPLING =
      110, /**< This parameter stores crosscoupling matrices which are used by
              the system after application of PARIMU_CALIB and before strapdown
              and EKF calculation */
  XCOMPAR_PARIMU_REFPOINTOFFSET =
      111, /**< This parameter stores the offset of the enclosure reference
              point with respect to the center of navigation. */
  XCOMPAR_PARIMU_BANDSTOP =
      112, /**< This parameter configures the bandstop filter which can be
              enabled for the accelerations and angular rates */
  XCOMPAR_PARIMU_COMPMETHOD =
      113, /**< This parameter configures the downsampling behavior prior to
              strapdown calculations if the prescaler value in PARSYS_PRESCALER
              is greater than 1. */
  XCOMPAR_PARIMU_ACCLEVERARM =
      114, /**< This parameter configures coordinates of the accelerometers in
              the body frame. These values are used to compensate for the size
              effect. */
  XCOMPAR_PARIMU_STRAPDOWNCONF =
      115, /**< This parameter configures the strapdown algorithm for different
              types of IMUs */
  XCOMPAR_PARIMU_C_ENC_IMU =
      116, /**< This parameter defines the rotation matrix from the iNAT
              standard enclosure frame (z down, x axis pointing from the plugs
              into the device) to a custom enclosure frame */
  XCOMPAR_PARIMU_RANGE =
      117, /**< This parameter defines the measurement range of the IMU */
  XCOMPAR_PARIMU_CALIBDATA = 118,

  /* GNSS */
  XCOMPAR_PARGNSS_PORT = 200, /**< This parameter is read-only and will be
                                 factory set during production */
  XCOMPAR_PARGNSS_BAUD =
      201, /**< This parameter is read-only and will be set during production */
  XCOMPAR_PARGNSS_ANTOFFSET = 204, /**< This parameter configures the GNSS lever
                                      arm of the primary or secondary antenna */
  XCOMPAR_PARGNSS_RTKMODE =
      207, /**< This parameter enables/disables the RTK mode */
  XCOMPAR_PARGNSS_AIDFRAME =
      209, /**< This parameter sets the GNSS aiding frame. The configured frame
              is valid for GNSS position and velocity aiding. */
  XCOMPAR_PARGNSS_RTCMV3AIDING =
      210, /**< This parameter can be used to input DGNSS correction data into
              the internal GNSS receiver over an iXCOM connection */
  XCOMPAR_PARGNSS_DUALANTMODE =
      211, /**< This parameter enables or disables GNSS dual antenna mode inside
              the GNSS module. */
  XCOMPAR_PARGNSS_LOCKOUTSYSTEM =
      212, /**< This parameter can be used to exclude certain satellite
              constellations from the GNSS solution computation. */
  XCOMPAR_PARGNSS_RTCMV3CONFIG = 213, /**< This parameter enables or disables
                                         the RTCMv3 forwarding module. */
  XCOMPAR_PARGNSS_NAVCONFIG = 214,
  XCOMPAR_PARGNSS_STDDEV = 215,
  XCOMPAR_PARGNSS_VOTER = 216,
  XCOMPAR_PARGNSS_MODEL =
      217, /**< This parameter gives a list of valid authorized models available
              and expiry date informa- tion */
  XCOMPAR_PARGNSS_VERSION =
      218, /**< This parameter contains GNSS receiver version information, both
              for hard- and software components */
  XCOMPAR_PARGNSS_RTKSOLTHR = 219,
  XCOMPAR_PARGNSS_TERRASTAR = 220,
  XCOMPAR_PARGNSS_REFSTATION =
      221, /**< This parameter configures the GNSS reference station feature. */
  XCOMPAR_PARGNSS_FIXPOS = 222, /**< This parameter configures the fixed
                                   position of the GNSS receiver */
  XCOMPAR_PARGNSS_POSAVE = 223, /**< This parameter configures the position
                                   averaging on the integrated GNSS receiver. */
  XCOMPAR_PARGNSS_CORPORTCFG =
      224, /**< This parameter configures the serial correction port of the
              internal GNSS receiver */
  XCOMPAR_PARGNSS_GATEWAYCFG = 225,
  XCOMPAR_PARGNSS_SWITCHER =
      226, /**< This parameter configures the GNSS switcher (internal/external
              GNSS receiver) */
  XCOMPAR_PARGNSS_ANTENNAPOWER =
      227, /**< This parameter  enables or disables the supply of electrical
              power from the internal power source of the receiver */
  XCOMPAR_PARGNSS_PPSCONTROL =
      228, /**< This parameter provides a method for controlling the polarity,
              period and pulse width of the PPS output  */
  XCOMPAR_PARGNSS_HDGOFFSET = 229, /**< This parameter adds on offset to the
                                      dual antenna GNSS heading */
  XCOMPAR_PARGNSS_STATUSCONFIG =
      230, /**< This parameter is used to configure the various status mask
              fields in the RXSTATUSEVENT log */
  XCOMPAR_PARGNSS_FWUPDATE =
      231, /**< This parameter is used to start a firmware update of the
              internal GNSS receiver */

  /* MAG */
  XCOMPAR_PARMAG_PORT = 300,   /**< This parameter configures the serial port of
                                  the magnetometer module. */
  XCOMPAR_PARMAG_PERIOD = 302, /**< This parameter configures the sampling rate
                                  of the magnetometer module. */
  XCOMPAR_PARMAG_MISALIGN =
      304, /**< This parameter changes the misalignment of the magnetometer
              reference frame with respect to the IMU reference frame */
  XCOMPAR_PARMAG_CAL = 307, /**< This parameter contains the calibration
                               parameters of the magnetometer. */
  XCOMPAR_PARMAG_CALSTATE =
      308, /**< This parameter holds the on-board magnetometer calibration
              procedure status of the 2D magnetometer calibration routine. */
  XCOMPAR_PARMAG_FOM =
      309, /**< This parameter contains the Figure Of Merit (FOM) of the 2D
              magnetometer calibration method. */
  XCOMPAR_PARMAG_CFG =
      310, /**< This parameter contains the magnetometer configuration items. */
  XCOMPAR_PARMAG_ENABLE =
      311, /**< This parameter enables/disables the magnetometer modules */

  /* ODO */
  XCOMPAR_PARODO_SCF = 1100, /**< This parameter defines the odometer scale
                                factor in [meter/tick]. */
  XCOMPAR_PARODO_TIMEOUT =
      1101, /**< This parameter defines the odometer timeout in [sec].  */
  XCOMPAR_PARODO_MODE = 1102, /**< This parameter defines the odometer mode and
                                 its related items. */
  XCOMPAR_PARODO_LEVERARM = 1103,    /**<  */
  XCOMPAR_PARODO_VELSTDDEV = 1104,   /**<  */
  XCOMPAR_PARODO_DIRECTION = 1105,   /**<  */
  XCOMPAR_PARODO_CONSTRAINTS = 1106, /**<  */
  XCOMPAR_PARODO_RATE = 1107,        /**<  */
  XCOMPAR_PARODO_THR = 1108,         /**<  */
  XCOMPAR_PARODO_EQEP = 1109,        /**< @deprecated */
  XCOMPAR_PARODO_INVERTER = 1110,    /**<  */
  XCOMPAR_PARODO_SWAP = 1111,        /**<  */
  XCOMPAR_PARODO_PPDDIVIDER = 1112,  /**<  */

  /* ARINC825 */
  XCOMPAR_ARINC825_PORT = 1200,
  XCOMPAR_ARINC825_BAUD = 1201,
  XCOMPAR_ARINC825_ENABLE = 1202,
  XCOMPAR_ARINC825_FRAMELIST = 1204,
  XCOMPAR_ARINC825_BUSRECOVERY = 1205,
  XCOMPAR_ARINC825_RESETSTAT = 1206,
  XCOMPAR_ARINC825_SCALEFACTOR = 1207,
  XCOMPAR_ARINC825_EVENTMASK = 1208,

  /* iMADC */
  XCOMPAR_MADC_ENABLE = 400,
  XCOMPAR_MADC_LEVERARM = 401,
  XCOMPAR_MADC_LOWPASS = 402,

  /* Data Recorder */
  XCOMPAR_REC_CONFIG = 600,
  XCOMPAR_REC_START = 603,
  XCOMPAR_REC_STOP = 604,
  XCOMPAR_REC_POWERLOGGER = 605,
  XCOMPAR_REC_SUFFIX = 606,
  XCOMPAR_REC_DISKSPACE = 607,
  XCOMPAR_REC_AUXILIARY = 608,
  XCOMPAR_REC_CURRENTFILE = 609,
  XCOMPAR_REC_LOGROTATE = 610,

  /* EKF */
  XCOMPAR_PAREKF_ALIGNMODE = 700,  /**< @deprecated */
  XCOMPAR_PAREKF_ALIGNTIME = 701,  /**< @deprecated */
  XCOMPAR_PAREKF_COARSETIME = 702, /**< @deprecated */
  XCOMPAR_PAREKF_VMP =
      703, /**< This parameter specifies a virtual measurement point. */
  XCOMPAR_PAREKF_AIDING = 704, /**< This parameter defines the used aiding
                                  sources of the extended Kalman filter. */
  XCOMPAR_PAREKF_DELAY =
      706, /**< This parameter defines the delay in [ms] of the delayed
              navigation module against the real-time navigator */
  XCOMPAR_PAREKF_STARTUP = 707, /**< @deprecated */
  XCOMPAR_PAREKF_HDGPOSTHR =
      708, /**< This parameter defines the thresholds of the Alignment Status
              and Position Accuracy fields of the global status  */
  XCOMPAR_PAREKF_SMOOTH =
      709, /**< This parameter defines the smoothing factor in [samples]. */
  XCOMPAR_PAREKF_ZUPTTHR = 712, /**< This parameter contains the configuration
                                   of the zero velocity detector */
  XCOMPAR_PAREKF_DEFPOS =
      714, /**< This parameter defines the default position */
  XCOMPAR_PAREKF_DEFHDG =
      715, /**< This parameter defines the default heading */
  XCOMPAR_PAREKF_OUTLIER =
      716, /**< This parameter defines the outlier rejection mask of the
              integrated Kalman filter. */
  XCOMPAR_PAREKF_POWERDOWN =
      717, /**< This parameter defines the power-down behavior of the system */
  XCOMPAR_PAREKF_EARTHRAD =
      718, /**< This parameter is read-only and contains the earth radii. */
  XCOMPAR_PAREKF_STOREDPOS =
      719, /**< This read-only parameter provides the stored position and the
              associated standard deviation. */
  XCOMPAR_PAREKF_ALIGNZUPTSTDDEV = 720, /**< @deprecated */
  XCOMPAR_PAREKF_POSAIDSTDDEVTHR =
      721, /**< This parameter configures the standard deviation threshold of
              the position aiding module */
  XCOMPAR_PAREKF_SCHULERMODE =
      722, /**< This parameter configures the system for Schuler Mode test. */
  XCOMPAR_PAREKF_STOREDATT =
      723, /**< This parameter is read-only and contains the stored attitude. */
  XCOMPAR_PAREKF_ODOMETER =
      724, /**< This parameter provides advanced odometer configuration options
              which can be used to accommodate for odometer signal
              abnormalities. */
  XCOMPAR_PAREKF_ODOBOGIE =
      725, /**< This parameter configures the odometer data processing module
              for railway applications */
  XCOMPAR_PAREKF_GNSSLEVERARMEST =
      726, /**< This parameter configures GNSS lever arm estimation. */
  XCOMPAR_PAREKF_GNSSAIDRATE = 727, /**< This parameter configures the GNSS
                                       aiding rates inside the Kalman filter. */
  XCOMPAR_PAREKF_KINALIGNTHR = 728, /**< @deprecated */
  XCOMPAR_PAREKF_GNSSPDOP = 729,    /**< This parameter configures the PDOP
                                       threshold inside the Kalman filter */
  XCOMPAR_PAREKF_DUALANTAID =
      730, /**< This parameter configures the dual antenna aiding inside the
              Kalman filter. */
  XCOMPAR_PAREKF_STARTUPV2 =
      731, /**< This parameter can be used to define the initialization
              behaviour of the Kalman filter. */
  XCOMPAR_PAREKF_MAGATTAID = 732, /**< This parameter configures the iMAG
                                     parameters inside the Kalman filter. */
  XCOMPAR_PAREKF_MADCAID = 733,   /**< This parameter configures the iMADC
                                     parameters inside the Kalman filter. */
  XCOMPAR_PAREKF_ALIGNMENT =
      734, /**< This parameter configures the alignment method of the INS */
  XCOMPAR_PAREKF_GRAVITYAIDING = 735, /**< This parameter configures the gravity
                                         aiding inside the Kalman filter. */
  XCOMPAR_PAREKF_FEEDBACK =
      736, /**< This parameter enables or disables the error compensation in the
              real-time navigation module */
  XCOMPAR_PAREKF_ZARU =
      737, /**< This parameter contains configures Zero Angular Rate Updates. */
  XCOMPAR_PAREKF_IMUCONFIG =
      738, /**< This parameter defines the initial standard deviations and the
              process noise model for the used inertial sensors. */
  XCOMPAR_PAREKF_ZUPTCALIB =
      739, /**< This parameter defines the calibration time for the Zero
              Velocity Update mechanism. */
  XCOMPAR_PAREKF_STATEFREEZE = 740, /**< This parameter contains the state
                                       freeze mask for realtime navigation. */
  XCOMPAR_PAREKF_RECOVERY =
      741, /**< This parameter contains the realignment condition bit mask. */
  XCOMPAR_PAREKF_STOREDVALSTDDEV = 742, /**< This parameter overwrites the
                                           stored position standard deviation */
  XCOMPAR_PAREKF_ODOCHECK = 743,     /**< This parameter configures the odometer
                                        outlier detection module */
  XCOMPAR_PAREKF_ODOREVDETECT = 744, /**< This parameter configures the odometer
                                        reverse detection module */
  XCOMPAR_PAREKF_PSEUDOPOS =
      745, /**< This parameter configures the pseudo position output */

  /* DATA */
  XCOMPAR_PARDAT_POS = 800,
  XCOMPAR_PARDAT_VEL = 801,
  XCOMPAR_PARDAT_IMU = 802,
  XCOMPAR_PARDAT_SYSSTAT = 803,

  /* XCOM */
  XCOMPAR_PARXCOM_SERIALPORT = 902,
  XCOMPAR_PARXCOM_NETCONFIG = 903,
  XCOMPAR_PARXCOM_LOGLISTE = 905,
  XCOMPAR_PARXCOM_AUTOSTART = 906,
  XCOMPAR_PARXCOM_NTRIP = 907,
  XCOMPAR_PARXCOM_POSTPROC = 908,
  XCOMPAR_PARXCOM_BROADCAST = 909,
  XCOMPAR_PARXCOM_UDPCONFIG = 910,
  XCOMPAR_PARXCOM_DUMPENABLE = 911,
  XCOMPAR_PARXCOM_MIGRATOR = 912,
  XCOMPAR_PARXCOM_TCPKEEPAL = 913,
  XCOMPAR_PARXCOM_CANGATEWAY = 914,
  XCOMPAR_PARXCOM_DEFAULTIP = 915,
  XCOMPAR_PARXCOM_ABDCONFIG = 916,
  XCOMPAR_PARXCOM_LOGLIST2 = 917,
  XCOMPAR_PARXCOM_CALPROC = 918,
  XCOMPAR_PARXCOM_CLIENT = 919,
  XCOMPAR_PARXCOM_FRAMEOUT = 920,
  XCOMPAR_PARXCOM_INTERFACE = 921,
  XCOMPAR_PARXCOM_MONITOR = 922,
  XCOMPAR_PARXCOM_QCONN = 923,
  XCOMPAR_PARXCOM_NETCHECK = 924,

  /* FPGA */
  XCOMPAR_PARFPGA_IMUSTATUSREG = 1000,
  XCOMPAR_PARFPGA_HDLCREG = 1001,
  XCOMPAR_PARFPGA_TIMINGREG = 1002,
  XCOMPAR_PARFPGA_TIMER = 1003,
  XCOMPAR_PARFPGA_INTERFACE = 1004,
  XCOMPAR_PARFPGA_CONTROLREG = 1005,
  XCOMPAR_PARFPGA_POWERUPTHR = 1006,
  XCOMPAR_PARFPGA_INTMAT245 = 1007,
  XCOMPAR_PARFPGA_TYPE = 1008,
  XCOMPAR_PARFPGA_GLOBALCONF = 1009,
  XCOMPAR_PARFPGA_HDLCPINMODE = 1010,
  XCOMPAR_PARFPGA_POWER = 1011,
  XCOMPAR_PARFPGA_ALARMTHR = 1012,
  XCOMPAR_PARFPGA_PPTCONFIG = 1013,
  XCOMPAR_PARFPGA_AUTOWAKEUP = 1014,
  XCOMPAR_PARFPGA_MCP23S08 = 1015,
  XCOMPAR_PARFPGA_CSAC = 1016,
  XCOMPAR_PARFPGA_GOBITMASK = 1017,
  XCOMPAR_PARFPGA_IMUPOWERMASK = 1018,
  XCOMPAR_PARFPGA_SYSPOWERMASK = 1019,
  XCOMPAR_PARFPGA_TRIGTIMEOUT = 1021,
  XCOMPAR_PARFPGA_HOLDOVERTIME = 1022,

  /* ARINC429 */
  XCOMPAR_PARARINC429_CONFIG = 1400,
  XCOMPAR_PARARINC429_LIST = 1401,

  /* I/O */
  XCOMPAR_PARIO_HW245 = 1500,
  XCOMPAR_PARIO_HW288 = 1501
};

/* XCOM PAR IDs */
enum XCOM_GNSSLEVERARM_IDX {
  PRIMARY_INTERN = 0x00,
  SECONDARY_INTERN = 0x01,
  PRIMARY_EXTERN = 0x02,
  SECONDARY_EXTERN = 0x03
};

/* GNSS Status */
#define XCOM_GNSSTATUS_INVALIDFIRMWARE (1 << 0)
#define XCOM_GNSSTATUS_ROMSTATUS (1 << 1)
#define XCOM_GNSSTATUS_SUPPLYVOLTAGE (1 << 2)
#define XCOM_GNSSTATUS_PLLRF1ERROR (1 << 3)
#define XCOM_GNSSTATUS_PLLRF2ERROR (1 << 4)
#define XCOM_GNSSTATUS_SWRESSOURCELIMIT (1 << 5)
#define XCOM_GNSSTATUS_INVALIDMODEL (1 << 6)
#define XCOM_GNSSTATUS_HWFAILURE (1 << 7)
#define XCOM_GNSSTATUS_TEMPERROR (1 << 8)
#define XCOM_GNSSTATUS_ANTENNAPOWERSTATUS (1 << 9)
#define XCOM_GNSSTATUS_ANTENNAOPEN (1 << 10)
#define XCOM_GNSSTATUS_ANTENNASHORTED (1 << 11)
#define XCOM_GNSSTATUS_CPUOVERLOAD (1 << 12)
#define XCOM_GNSSTATUS_COM1BUFFEROVERUN (1 << 13)
#define XCOM_GNSSTATUS_COM2BUFFEROVERUN (1 << 14)
#define XCOM_GNSSTATUS_COM3BUFFEROVERUN (1 << 15)
#define XCOM_GNSSTATUS_ALMANACFLAG (1 << 16)
#define XCOM_GNSSTATUS_POSSOLUTIONINVALID (1 << 17)
#define XCOM_GNSSTATUS_VOLTAGESUPPLY (1 << 21)
#define XCOM_GNSSTATUS_UPDATEINPROGRESS (1 << 31)

/**
 * XCOM Extended System Status
 *
 * Note: Once a sensor failure related to bit numbers 4-11 (Accelerometer and
 * Gyro overrange, invalid calibration) or bit 18 (EKF error) is set, it will
 * not be reset during mission. The bit numbers 4-11 and 18 can be reset via a
 * re-alignment.
 */
typedef struct {
  uint32_t IMUINVALID : 1;  /**< IMU data are invalid. This critical error can
                               occur if the calibration is invalid or the IMU
                               temperature is out of range */
  uint32_t IMUCRCERROR : 1; /**< CRC error in the internal communication */
  uint32_t IMUTIMEOUTERROR : 1; /**< IMU timeout bit is set if the main CPU does
                                   not receive any data from the IMU PCB */
  uint32_t IMUSAMPLELOST : 1;   /**< IMU sample lost. This can be done if a CRC
                                   error occurred */
  uint32_t ACCXOR : 1; /**< IMU’s x-accelerometer has detected an over range. */
  uint32_t ACCYOR : 1; /**< IMU’s y-accelerometer has detected an over range. */
  uint32_t ACCZOR : 1; /**< IMU’s z-accelerometer has detected an over range. */
  uint32_t RESERVED : 1; /**< Reserved for further use */
  uint32_t OMGXOR : 1;   /**< IMU’s x-gyroscope has detected an over range. */
  uint32_t OMGYOR : 1;   /**< IMU’s y-gyroscope has detected an over range. */
  uint32_t OMGZOR : 1;   /**< IMU’s z-gyroscope has detected an over range. */
  uint32_t INVALIDCALIBRATION : 1; /**< IMU’s calibration is invalid */
  uint32_t BITFAIL : 1;     /**< Built-In-Test failure. If this critical error
                               occurred the data should not be used */
  uint32_t DEFCONFIG : 1;   /**< This bit is set if the system cannot find a
                               configuration file. In this case the default
                               configuration will be loaded */
  uint32_t GNSSINVALID : 1; /**< GNSS solution is invalid */
  uint32_t ZUPTCALIBRUNNING : 1; /**< Zero Velocity Update is currently
                                    calibrating */
  uint32_t GNSSCRCERR : 1;  /**< CRC error in the internal communication. */
  uint32_t GNSSTIMEOUT : 1; /**< GNSS timeout bit is set if the main CPU does
                               not receive any data from the GNSS receiver. */
  uint32_t EKFERROR : 1; /**< Error inside the extended Kalman filter. If this
                            bit is set, the solution might be invalid. */
  uint32_t EKFSAVEDPOSERR : 1; /**< The INS could not load the last saved
                                  position for the initial alignment or the
                                  stored position is invalid. */
  uint32_t EKFSAVEDHDGERR : 1; /**< The INS could not load the last saved
                                  heading for the initial alignment or the
                                  stored heading is invalid. */
  uint32_t MAGTIMEOUT : 1; /**< MAG timeout bit is set if the main CPU does not
                              receive any data from the magnetometer. */
  uint32_t
      MADCTIMEOUT : 1;    /**< MADC timeout bit is set if the main CPU does not
                             receive any data from the air data computer. */
  uint32_t RESERVED1 : 1; /**< Reserved for further use */
  uint32_t INMOTION : 1;  /**< System is in-motion */
  uint32_t ODOPLAUSIBILITYERROR : 1; /**< Odometer plausibility check failed */
  uint32_t ODOHARDWAREERROR : 1;     /**< Error inside the odometer module */
  uint32_t WAITFORFORCEDVAL : 1;     /**< The system is configured for forced
                                        position alignment. The bit is set until
                                        initial values are received */
  uint32_t PPSLOST : 1; /**< If this bit is set the system recognized a timeout
                           of the trigger signal. This may result to in accuracy
                           in timestamp. */
  uint32_t LIMACCURACY : 1; /**< Limited accuracy, due to: IMU CRC Error, IMU
                               Timeout, Invalid calibration, EKF error */
  uint32_t RECENABLE : 1;   /**< Internal data recorder enabled/disabled */
  uint32_t FPGANOGO : 1;    /**< FPGA Error */
} XCOMSystemStatus;

/**
 * The extended Kalman filter aiding status contains information about
 * alignment, measurement updates and outlier detection.
 */
typedef struct {
  uint32_t POSLLH_UPDATE : 1; /**< Position aiding with GNSS data in the current
                                 time step */
  uint32_t POSLLH_LAT_OUTLIER : 1; /**< GNSS latitude outlier detected */
  uint32_t POSLLH_LON_OUTLIER : 1; /**< GNSS longitude outlier detected */
  uint32_t POSLLH_ALT_OUTLIER : 1; /**< GNSS altitude outlier detected */
  uint32_t VNED_UPDATE : 1; /**< Velocity aiding with GNSS data in the current
                               time step. */
  uint32_t VNED_VN_OUTLIER : 1; /**< GNSS VNorth outlier detected */
  uint32_t VNED_VE_OUTLIER : 1; /**< GNSS VEast outlier detected */
  uint32_t VNED_VD_OUTLIER : 1; /**< GNSS VDown outlier detected */
  uint32_t HEIGHT_UPDATE : 1;   /**< External height aiding in the current time
                                   step. */
  uint32_t HEIGHT_OUTLIER : 1;  /**< Height outlier detected */
  uint32_t BAROHEIGHT_UPDATE : 1;  /**< Height aiding from barometer in the
                                      current time step. */
  uint32_t BAROHEIGHT_OUTLIER : 1; /**< Baro-Altitude outlier detected */
  uint32_t VBDY_UPDATE : 1; /**< External velocity aiding (body velocity) in the
                               current time step. */
  uint32_t VBDY_VX_OUTLIER : 1; /**< Vbody_x outlier detected */
  uint32_t VBDY_VY_OUTLIER : 1; /**< Vbody_y outlier detected */
  uint32_t VBDY_VZ_OUTLIER : 1; /**< Vbody_z outlier detected */
  uint32_t VODO_UPDATE : 1;  /**< Velocity aiding from odometer in the current
                                time step. */
  uint32_t VODO_OUTLIER : 1; /**< Odometer velocity outlier detected */
  uint32_t VAIR_UPDATE : 1;  /**< Velocity aiding from air data computer in the
                                current time step. */
  uint32_t VAIR_OUTLIER : 1; /**< Air speed outlier detected */
  uint32_t HDGT_UPDATE : 1;  /**< Heading aiding in the current time step. */
  uint32_t HDGT_OUTLIER : 1; /**< Heading outlier detected */
  uint32_t HDGM_UPDATE : 1; /**< Heading aiding from magnetometer in the current
                               time step. */
  uint32_t HDGM_OUTLIER : 1;    /**< Magnetic heading outlier detected */
  uint32_t MAGFIELD_UPDATE : 1; /**< Magnetic field aiding from magnetometer in
                                   the current time step. */
  uint32_t MAGFIELD_HX_OUTLIER : 1; /**< Magx outlier detected */
  uint32_t MAGFIELD_HY_OUTLIER : 1; /**< Magy outlier detected */
  uint32_t MAGFIELD_HZ_OUTLIER : 1; /**< Magz outlier detected */
  uint32_t PSEUDORANGE_UPDATE : 1;  /**< Pseudorange aiding from GNSS in the
                                       current time step. */
  uint32_t RANGERATE_UPDATE : 1; /**< Range rate aiding from GNSS in the current
                                    time step */
  uint32_t TIME_UPDATE : 1;      /**< Time update in the current time step */
  uint32_t
      ZUPT_UPDATE : 1; /**< Zero velocity update in the current time step */
} XCOMEkfAidingStatLo;

typedef struct {
  uint32_t RTKLLH_UPD : 1; /**< RTK position aiding from GNSS in the current
                              time step */
  uint32_t RTKLLH_LON_OUTLIER : 1; /**< RTK latitude outlier detected */
  uint32_t RTKLLH_LAT_OUTLIER : 1; /**< RTK longitude outlier detected  */
  uint32_t RTKLLH_ALT_OUTLIER : 1; /**< RTK altitude outlier detected  */
  uint32_t
      BSLXYZ_UPDATE : 1; /**< Dual antenna aiding in the current time step */
  uint32_t BSLXYZ_OUTLIER : 1; /**< Dual antenna outlier detected */
  uint32_t
      COG_UPDATE : 1; /**< Course over ground update in the current timestep */
  uint32_t COG_OUTLIER : 1;    /**< Course over ground outlier detected */
  uint32_t TDCP_UPDATE : 1;    /**< TDCP aiding in the current time step */
  uint32_t TDCP_DD_UPDATE : 1; /**< TDCP-DD aiding in the current time step */
  uint32_t ODO_ALONGTRK_UPDATE : 1;  /**< Odometer track aiding in the current
                                        time step */
  uint32_t ODO_ALONGTRK_OUTLIER : 1; /**< Odometer track outlier detected */
  uint32_t ODO_CONST_UPDATE : 1;  /**< Odometer constraint aiding in the current
                                     time step */
  uint32_t ODO_CONST_OUTLIER : 1; /**< Odometer constraint outlier detected */
  uint32_t GRAV_UPDATE : 1;       /**< Gravity update */
  uint32_t GRAV_OUTLIER : 1;      /**< Gravity outlier detected */
  uint32_t EXTPOS_UPD : 1;        /**< External position update */
  uint32_t EXTPOS_OUTLIER : 1;    /**< External position outlier detected */
  uint32_t EXTVEL_UPD : 1;        /**< External velocity update */
  uint32_t EXTVEL_OUTLIER : 1;    /**< External velocity outlier detected */
  uint32_t ZARU : 1;              /**< Zero Angular Rate Update */
  uint32_t RESERVED : 3;          /**< Reserved for further use */
  uint32_t WAHBA_UPDATE : 1;   /**< Vector direction update in coarse alignment
                                  filter */
  uint32_t WAHBA_FILTER : 1;   /**< Coarse alignment filter active */
  uint32_t FILTERMODE_1 : 1;   /**< EKF mode 1 */
  uint32_t FILTERMODE_2 : 1;   /**< EKF mode 2 */
  uint32_t WAHBA_INTERNAL : 1; /**< Coarse alignment filter active */
  uint32_t LEVELLING_COMPLETE : 1; /**< Levelling has completed */
  uint32_t ALIGN_COMPLETE : 1;     /**< Stationary alignment has completed */
  uint32_t INITPOS_SET : 1;        /**< The initial position is set */
} XCOMEkfAidingStatHi;

/* PAREKF_FEEDBACK */
#define XCOM_EKF_FEEDBACK_POSITION (1 << 0)
#define XCOM_EKF_FEEDBACK_VELOCITY (1 << 1)
#define XCOM_EKF_FEEDBACK_ATTITUDE (1 << 2)
#define XCOM_EKF_FEEDBACK_SENSORERROR (1 << 3)

/* PAREKF_STATEFREEZE */
#define XCOM_EKF_FREEZE_POSITION (1 << 0)
#define XCOM_EKF_FREEZE_VELOCITY (1 << 1)
#define XCOM_EKF_FREEZE_ATTITUDE (1 << 2)
#define XCOM_EKF_FREEZE_HEIGHT (1 << 3)

/* PARDAT_SYSSTAT */
#define XCOM_PARDAT_SYSSTAT_IMU (1 << 0)
#define XCOM_PARDAT_SYSSTAT_GNSS (1 << 1)
#define XCOM_PARDAT_SYSSTAT_MAG (1 << 2)
#define XCOM_PARDAT_SYSSTAT_MADC (1 << 3)
#define XCOM_PARDAT_SYSSTAT_EKFAIDING (1 << 4)
#define XCOM_PARDAT_SYSSTAT_EKFGENERAL (1 << 5)
#define XCOM_PARDAT_SYSSTAT_ADDSTATUS (1 << 6)
#define XCOM_PARDAT_SYSSTAT_SERVICE (1 << 7)
#define XCOM_PARDAT_SYSSTAT_REMALIGNTIME (1 << 8)

/* IO */
#define XCOM_PARIO_HW245_EVENIN_ENABLE (1 << 0) /**< SYNC_1 */
#define XCOM_PARIO_HW245_PPT_ENABLE (1 << 1)    /**< SYNC_2 */
#define XCOM_PARIO_HW245_TOV_ENABLE (1 << 2)    /**< SYNC_3 */
#define XCOM_PARIO_HW245_PPS_ENABLE (1 << 3)    /**< SYNC_4 */
#define XCOM_PARIO_HW245_EVENIN_INVERT                                         \
  (1 << 4) /**< INVERT --> FALLING EDGE                                        \
            */

/* Magnetometer1 aiding mode */
#define XCOM_MAGAIDMODE_INITMODE 0
#define XCOM_MAGAIDMODE_INTERVALMODE 1
#define XCOM_MAGAIDMODE_AUTOMODE 2
#define XCOM_MAGAIDMODE_NAVAIDMODE 3

/* CAN Status */
#define XCOM_CANSTATUS_ERRMASK_TX_TIMEOUT                                      \
  0x00000001U /**< TX timeout (by driver) */
#define XCOM_CANSTATUS_ERRMASK_LOSTARB 0x00000002U /**< lost arbitration    */
#define XCOM_CANSTATUS_ERRMASK_CRTL 0x00000004U    /**< controller problems */
#define XCOM_CANSTATUS_ERRMASK_PROT 0x00000008U    /**< protocol violations */
#define XCOM_CANSTATUS_ERRMASK_TRX 0x00000010U     /**< transceiver status  */
#define XCOM_CANSTATUS_ERRMASK_ACK                                             \
  0x00000020U /**< received no ACK on transmission */
#define XCOM_CANSTATUS_ERRMASK_BUSOFF 0x00000040U   /**< bus off */
#define XCOM_CANSTATUS_ERRMASK_BUSERROR 0x00000080U /**< bus error */
#define XCOM_CANSTATUS_ERRMASK_RESTARTED                                       \
  0x00000100U /**< controller restarted */

#define XCOM_CANSTATUS_CRTL_UNSPEC 0x00      /**< unspecified */
#define XCOM_CANSTATUS_CRTL_RX_OVERFLOW 0x01 /**< RX buffer overflow */
#define XCOM_CANSTATUS_CRTL_TX_OVERFLOW 0x02 /**< TX buffer overflow */
#define XCOM_CANSTATUS_CRTL_RX_WARNING                                         \
  0x04 /**< reached warning level for RX errors */
#define XCOM_CANSTATUS_CRTL_TX_WARNING                                         \
  0x08 /**< reached warning level for TX errors */
#define XCOM_CANSTATUS_CRTL_RX_PASSIVE                                         \
  0x10 /**< reached error passive status RX */
#define XCOM_CANSTATUS_CRTL_TX_PASSIVE                                         \
  0x20 /**< reached error passive status TX */

#define XCOM_CANSTATUS_PROT_UNSPEC 0x00   /**< unspecified */
#define XCOM_CANSTATUS_PROT_BIT 0x01      /**< single bit error */
#define XCOM_CANSTATUS_PROT_FORM 0x02     /**< frame format error */
#define XCOM_CANSTATUS_PROT_STUFF 0x04    /**< bit stuffing error */
#define XCOM_CANSTATUS_PROT_BIT0 0x08     /**< unable to send dominant bit */
#define XCOM_CANSTATUS_PROT_BIT1 0x10     /**< unable to send recessive bit */
#define XCOM_CANSTATUS_PROT_OVERLOAD 0x20 /**< bus overload */
#define XCOM_CANSTATUS_PROT_ACTIVE 0x40   /**< active error announcement */
#define XCOM_CANSTATUS_PROT_TX 0x80       /**< error occurred on transmission */

#define XCOM_CANSTATUS_PROT_LOC_UNSPEC 0x00 /**< unspecified */
#define XCOM_CANSTATUS_PROT_LOC_SOF 0x03    /**< start of frame */
#define XCOM_CANSTATUS_PROT_LOC_ID28_21                                        \
  0x02 /**< ID bits 28 - 21 (SFF: 10 - 3) */
#define XCOM_CANSTATUS_PROT_LOC_ID20_18                                        \
  0x06                                    /**< ID bits 20 - 18 (SFF: 2 - 0 )*/
#define XCOM_CANSTATUS_PROT_LOC_SRTR 0x04 /**< substitute RTR (SFF: RTR) */
#define XCOM_CANSTATUS_PROT_LOC_IDE 0x05  /**< identifier extension */
#define XCOM_CANSTATUS_PROT_LOC_ID17_13 0x07 /**< ID bits 17-13 */
#define XCOM_CANSTATUS_PROT_LOC_ID12_05 0x0F /**< ID bits 12-5 */
#define XCOM_CANSTATUS_PROT_LOC_ID04_00 0x0E /**< ID bits 4-0 */
#define XCOM_CANSTATUS_PROT_LOC_RTR 0x0C     /**< RTR */
#define XCOM_CANSTATUS_PROT_LOC_RES1 0x0D    /**< reserved bit 1 */
#define XCOM_CANSTATUS_PROT_LOC_RES0 0x09    /**< reserved bit 0 */
#define XCOM_CANSTATUS_PROT_LOC_DLC 0x0B     /**< data length code */
#define XCOM_CANSTATUS_PROT_LOC_DATA 0x0A    /**< data section */
#define XCOM_CANSTATUS_PROT_LOC_CRC_SEQ 0x08 /**< CRC sequence */
#define XCOM_CANSTATUS_PROT_LOC_CRC_DEL 0x18 /**< CRC delimiter */
#define XCOM_CANSTATUS_PROT_LOC_ACK 0x19     /**< ACK slot */
#define XCOM_CANSTATUS_PROT_LOC_ACK_DEL 0x1B /**< ACK delimiter */
#define XCOM_CANSTATUS_PROT_LOC_EOF 0x1A     /**< end of frame */
#define XCOM_CANSTATUS_PROT_LOC_INTERM 0x12  /**< intermission */

#define XCOM_CANSTATUS_TRX_UNSPEC 0x00             /**< 0000 0000 */
#define XCOM_CANSTATUS_TRX_CANH_NO_WIRE 0x04       /**< 0000 0100 */
#define XCOM_CANSTATUS_TRX_CANH_SHORT_TO_BAT 0x05  /**< 0000 0101 */
#define XCOM_CANSTATUS_TRX_CANH_SHORT_TO_VCC 0x06  /**< 0000 0110 */
#define XCOM_CANSTATUS_TRX_CANH_SHORT_TO_GND 0x07  /**< 0000 0111 */
#define XCOM_CANSTATUS_TRX_CANL_NO_WIRE 0x40       /**< 0100 0000 */
#define XCOM_CANSTATUS_TRX_CANL_SHORT_TO_BAT 0x50  /**< 0101 0000 */
#define XCOM_CANSTATUS_TRX_CANL_SHORT_TO_VCC 0x60  /**< 0110 0000 */
#define XCOM_CANSTATUS_TRX_CANL_SHORT_TO_GND 0x70  /**< 0111 0000 */
#define XCOM_CANSTATUS_TRX_CANL_SHORT_TO_CANH 0x80 /**< 1000 0000 */

/* eQEP Configuration */
#define XCOM_EQEP_MODE_QUADRATURE                                              \
  (1 << 0) /**< 1: Quadrature mode; 			0: Direction mode */
#define XCOM_EQEP_MODE_EXTCLOCKRATE                                            \
  (1 << 1) /**< 1: 1x resolution;   			0: 2x resolution                        \
            * (direction mode                                                  \
            */
#define XCOM_EQEP_MODE_SWAPINPUTS                                              \
  (1 << 2) /**< 1: Clock inputs are swapped; 0: Clock inputs are not swapped*/
#define XCOM_EQEP_MODE_QAP                                                     \
  (1 << 3) /**< 1: Negates A input;			0: No effect */
#define XCOM_EQEP_MODE_QBP                                                     \
  (1 << 4) /**< 1: Negates B input;			0: No effect */

/* PARDAT_IMU */
#define XCOM_PARDAT_IMU_IMURAW 0
#define XCOM_PARDAT_IMU_IMUCORR 1
#define XCOM_PARDAT_IMU_IMUCOMP 2
#define XCOM_PARDAT_IMU_IMUCORRFILTERED 3
#define XCOM_PARDAT_IMU_IMUCOMPFILTERED 4

/* ARINC429 Status */
#define XCOM_ARINC429STAT_BIT0_DONE (1 << 0)
#define XCOM_ARINC429STAT_BIT1_DONE (1 << 1)
#define XCOM_ARINC429STAT_BIT2_DONE (1 << 2)
#define XCOM_ARINC429STAT_BIT3_DONE (1 << 3)
#define XCOM_ARINC429STAT_TFLAG3 (1 << 8)
#define XCOM_ARINC429STAT_RFLAG3 (1 << 9)
#define XCOM_ARINC429STAT_TFLAG2 (1 << 10)
#define XCOM_ARINC429STAT_RFLAG2 (1 << 11)
#define XCOM_ARINC429STAT_TFLAG1 (1 << 12)
#define XCOM_ARINC429STAT_RFLAG1 (1 << 13)
#define XCOM_ARINC429STAT_TFLAG0 (1 << 14)
#define XCOM_ARINC429STAT_RFLAG0 (1 << 15)

#define XCOM_OEM6XX_HWMON_ENTRIES 16

/* FPGA Power Switches */
#define XCOM_FPGAPOWER_GPS_5V (1 << 0)
#define XCOM_FPGAPOWER_VMS (1 << 1)
#define XCOM_FPGAPOWER_ISO (1 << 2)
#define XCOM_FPGAPOWER_IMU (1 << 3)
#define XCOM_FPGAPOWER_OEM628 (1 << 4)
#define XCOM_FPGAPOWER_OEM615 (1 << 5)

/* ARINC825 Event Based Frames */
#define ARINC825_EVENTMASK_ALIGNBSL (1 << 0)
#define ARINC825_EVENTMASK_PPS (1 << 1)

/* CSAC */
#define XCOM_CSACMODE_ENABLE (1 << 0)
#define XCOM_CSACMODE_ENABLEATUNE (1 << 1)
#define XCOM_CSACMODE_PPSAUTOSYNC (1 << 2)
#define XCOM_CSACMODE_PPSDISCIPLINE (1 << 3)
#define XCOM_CSACMODE_STARTSYNC (1 << 4)
#define XCOM_CSACMODE_GNSSAUTOMODE (1 << 5)

// TODO: Should be replayed by an enum in the future to guarantee compatibility
// with older compiler versions
enum XCOMResp {
  OK = 0,
  INVALIDPAR = 1,
  INVALIDCRC = 2,
  INVALIDLOG = 3,
  INVALIDRATE = 4,
  INVALIDPORT = 5,
  INVALIDCMD = 6,
  INVALIDID = 7,
  INVALIDCHANNEL = 8,
  PAROUTOFRANGE = 9,
  LOGEXISTS = 10,
  INVALIDTRIGGER = 11,
  INTERNALERROR = 12,
  PATHEXISTS = 13,
  UARTBUFFEROVERFLOW = 14,
  STOREDVALUEHASNOTBEENSET = 15,
  MAGERRORALIGN = 16,
  CONFIGERROR = 17,
  PARAMCANNOTBECHANGED = 18,
  GNSSPORTALREADYINUSE = 19,
  USERDEFINEDMESSAGE = 20,
  RECORDERNOTRUNNING = 21,
  INVALIDLENGTH = 22,
  NOTIMPLEMENTED = 23,
  DEPRECATED = 24
};

typedef struct __attribute__((__packed__)) {
  uint8_t sync;          /**< Synchronization character (is always set to 0x7E;
                            XCOM_SYNC_BYTE) */
  uint8_t msg_id;        /**< iXCOM message ID. iXCOM distinguishes between four
                            different types of message IDs (ordinary message IDs;
                            command ID; parameter ID, response ID) */
  uint8_t frame_counter; /**< The frame counter counts from 0 to 255 and starts
                            at 0 again after it has reached 255. For messages,
                            this can be used to detect lost messages. For
                            parameters and commands, the system response to an
                            input parameter or command will contain the same
                            frame counter as the input. */
  uint8_t trigger_source; /**< See <iXCOM LOG command trigger type> */
  uint16_t msg_len;  /**< Length of the overall message (from sync byte to crc16
                        field), depends on the message log (num- ber of bytes) */
  uint16_t gps_week; /**< GPS week number */
  uint32_t gps_time_sec;  /**< GPS time of week - integer part in [sec] */
  uint32_t gps_time_usec; /**< GPS time of week - fractional part in [μs] */
} XCOMHeader;

/**
 * The global status is a summarized status word that contains a combination of
 * several status bits. This status word should be used to evaluate the data
 * integrity and is attached as part of the footer at the end of every message.
 * If an error occurs, the detailed status should be read.
 */
typedef struct {
  uint16_t HWERROR : 1;  /**< Unexpected hardware error. */
  uint16_t COMERROR : 1; /**< Internal communication error between the
                            individual components. */
  uint16_t
      NAVERROR : 1; /**< Internal error in the navigation solution. If this bit
                       is Critical set the INS solution should not be used. */
  uint16_t CALERROR : 1;     /**< Internal error in the calibration table. */
  uint16_t GYROVERRANGE : 1; /**< Gyro over range error. To see which axis is
                                affected, the extended system status should be
                                triggered. */
  uint16_t ACCOVERRAGE : 1;  /**< Acceleration over range error. To see which
                                axis is affected, the extended system status
                                should be triggered. */
  uint16_t GNSSINVALID : 1;  /**< GNSS solution is invalid */
  uint16_t STANDBY : 1;      /**< System stays in standby mode. */
  uint16_t DYNAMICALIGN : 1; /**< The INS is in dynamic alignment mode */
  uint16_t TIMEINVALID : 1;  /**< The message time tag in the header represents
                                internal system time and has not been set from
                                GNSS. */
  uint16_t NAVMODE : 1;      /**< The INS has reached the fine-heading mode */
  uint16_t AHRSMODE : 1;     /**< The INS has reached the AHRS mode*/
  uint16_t
      ALIGNMODE : 2; /**< Alignment mode
                                                      0: LEVELLING: This
                        solution status is set during the levelling phase. 1:
                        STATIONARY_ALIGN: This solution status is set during
                        stationary alignment of the INS. 2: ALIGNMENT_COMPLETE:
                        This solution status is set if the alignment has
                        finished and the standard deviation of the estimated
                        heading is larger than THR_HEADING (set through
                        parameter PAREKF_HDGPOSTHR). 3: HEADING_GOOD: This
                        solution status is set if the alignment has finished and
                        the standard deviation of the estimated heading is lower
                        than THR_HEADING (set through parameter
                        PAREKF_HDGPOSTHR).
                                              */
  uint16_t
      POSMODE : 2; /**< Position mode:
                                           0: POS_BAD_ACCURACY: This solution
                      status is set during alignment, if no valid position
                      solution is available. 1: POS_MEDIUM_ACCURACY: This
                      solution status is set, if the INS has calculated a valid
                      position solution and the standard deviation of the
                      estimated position is lower than THR_POS_MED AND larger
                      than THR_POS_HIGH (set through parameter
                      PAREKF_HDGPOSTHR). 2: POS_HIGH_ACCURACY: This solution
                      status is set if the INS has calculated a valid position
                      solution and the standard deviation of the estimated
                      position is lower than THR_POS_HIGH (set through parameter
                      PAREKF_HDGPOSTHR). 3: POS_UNDEFINED: The position accuracy
                      is undefined.
                                    */
} XCOMGlobalStatus;

/**
 * Global status alignment mode
 */
enum XCOMGlobalStatusAlignmode {
  Levelling = 0, /**< This solution status is set during the levelling phase */
  Aligning = 1,  /**< This solution status is set during stationary alignment of
                    the INS */
  AlignmentComplete =
      2, /**< This solution status is set if the alignment has finished and the
            standard deviation of the estimated heading is larger than
            THR_HEADING (set through parameter PAREKF_HDGPOSTHR) */
  AlignmentGood =
      3 /**< This solution status is set if the alignment has finished and the
           standard deviation of the estimated heading is lower than THR_HEADING
           (set through parameter PAREKF_HDGPOSTHR) */
};

/**
 * Global status position mode
 */
enum XCOMGlobalStatusPosMode {
  PosBad = 0,    /**< This solution status is set during alignment, if no valid
                    position solution is available */
  PosMadium = 1, /**< This solution status is set, if the INS has calculated a
                    valid position solution and the standard deviation of the
                    estimated position is lower than THR_POS_MED AND larger than
                    THR_POS_HIGH (set through parameter PAREKF_HDGPOSTHR) */
  PosHigh = 2,   /**< This solution status is set if the INS has calculated a
                    valid position solution and the standard deviation of the
                    estimated position is lower than THR_POS_HIGH (set through
                    parameter PAREKF_HDGPOSTHR) */
  PosUndefined = 3 /**< The position accuracy is undefined */
};

typedef struct __attribute__((__packed__)) {
  union { /**< The global status is a summarized status word that contains a
             combination of several status bits. This status word should be used
             to evaluate the data integrity and is attached as part of the
             footer at the end of every message. If an error occurs, the
             detailed status should be read. */
    XCOMGlobalStatus bits;
    uint16_t value;
  } global_status;
  uint16_t crc16; /**< Always sent, calculated over all bytes starting with
                     synchronization byte. */
} XCOMFooter;

/**
 * ******************************************************************************************
 * XCOM message definition
 * ******************************************************************************************
 */

typedef struct __attribute__((__packed__)) {
  XCOMHeader header;
  uint8_t payload_buffer[XCOM_MAX_MESSAGE_LENGTH + sizeof(XCOMFooter)];
} XCOMGeneric;

typedef struct __attribute__((__packed__)) {
  uint8_t port;        /**< External serial port number */
  uint8_t reserved[3]; /**< Reserved for further use */
} XCOMPassthroughHeader;

/**
 * This message contains different time related data, regarding system,
 * IMU, PPS and GNSS.
 */
typedef struct __attribute__((__packed__)) {
  XCOMHeader header;
  double sys_time;           /**< System time in [sec] */
  double imu_interval;       /**< IMU interval time in [sec] */
  double time_since_pps;     /**< Time since last PPS in [sec] */
  double pps_imu_time;       /**< PPS IMU time in [sec] */
  double pps_gnss_time;      /**< PPS GNSS time in [sec] */
  double gnss_bias;          /**< GNSS time bias in [sec] */
  double gnss_bias_smoothed; /**< GNSS time bias smoothed in [sec] */
  XCOMFooter footer;
} XCOMmsg_TIME;

/**
 * The IMURAW message contains the calibrated measurements from the IMU.
 * This message is not corrected for IMU errors estimated by the EKF.
 * The IMU misalignment defined in PARIMU_MISALIGN is applied to the IMURAW log.
 */
typedef struct __attribute__((__packed__)) {
  XCOMHeader header;
  float acc[3]; /**< Calibrated acceleration along IMU x-, y- and z-axis in
                   [m/s2] */
  float omg[3]; /**< Calibrated angular rate along IMU x-, y- and z-axis in
                   [rad/s] */
  XCOMFooter footer;
} XCOMmsg_IMURAW;

/**
 * The IMUCORR message contains the calibrated measurements from the IMU.
 * This log is corrected for IMU errors estimated by the EKF.
 * The IMU misalignment defined in PARIMU_MISALIGN is applied to the IMUCORR
 * log.
 */
typedef struct __attribute__((__packed__)) {
  XCOMHeader header;
  float acc[3]; /**< Calibrated and corrected acceleration along IMU x-, y- and
                   z-axis in [m/s2] */
  float omg[3]; /**< Calibrated and corrected angular rate along IMU x-, y- and
                   z-axis in [rad/s] */
  XCOMFooter footer;
} XCOMmsg_IMUCORR;

/**
 * The IMUCOMP message contains the calibrated measurements from the IMU
 * (IMUCORR), additionally compensated for earth rate and gravity. The IMU
 * misalignment defined in PARIMU_MISALIGN is applied to the IMUCOMP log.
 */
typedef struct __attribute__((__packed__)) {
  XCOMHeader header;
  float acc[3]; /**< Calibrated and compensated acceleration along IMU x-, y-
                   and z-axis in [m/s2] */
  float omg[3]; /**< Calibrated and compensated angular rate along IMU x-, y-
                   and z-axis in [rad/s] */
  XCOMFooter footer;
} XCOMmsg_IMUCOMP;

/**
 * The OMGINT message contains the integrated angular rates for each axis
 * seperately.
 */
typedef struct __attribute__((__packed__)) {
  XCOMHeader header;
  float omg_int[3]; /**< Integrated angular rate for x-, y- and z- axis in [rad]
                     */
  float integration_time; /**< Integration time in [sec] */
  XCOMFooter footer;
} XCOMmsg_OMGINT;

/**
 * This message contains the data needed for the strapdown rotation test
 * defined by Savage. This still needs to be tested.
 */
typedef struct __attribute__((__packed__)) {
  XCOMHeader header;
  double acc_ned[3]; /**< Measured acceleration in NED frame in [m/s2] */
  XCOMFooter footer;
} XCOMmsg_INSROTTEST;

/**
 * This message contains special data values (e.g. raw data before rotation)
 * used for IMU calibration.
 */
typedef struct __attribute__((__packed__)) {
  XCOMHeader header;
  float acc_raw[3]; /**< Uncalibrated Accelerometer output for x-, y- and z-axis
                       in LSBs */
  float omg_raw[3]; /**< Uncalibrated Gyroscope output for x-, y- and z-axis in
                       LSBs */
  double acc_cal[3]; /**< Calibrated Accelerometer output for x-, y- and z-axis
                        in m/s2 */
  double omg_cal[3]; /**< Calibrated Gyroscope output for x-, y- and z-axis in
                        rad/s */
  double avg_duration_imu; /**< Averaging duration as sum of nominal IMU
                              intervals */
  double q_nb[4];          /**< Quaterion elements, used within NED frame */
  double pos_llh[3]; /**< Longitude, latitude and altitude position within NED
                        frame in [rad] and [m] */
  double vel_ned[3]; /**< Velocity within NED frame */
  float temperature[XCOM_MAX_NUMBER_OF_TEMPERATURES]; /**< Average IMU dependent
                                                         system temperature
                                                         array */
  uint32_t sys_stat;                                  /**< System Status  */
  uint32_t ekf_stat_low; /**< Extended Kalman Filter Aiding Status (Low word) */
  uint32_t ekf_stat_hi; /**< Extended Kalman Filter Aiding Status (High word) */
  uint32_t imu_status[6]; /**< IMU Status */
  XCOMFooter footer;
} XCOMmsg_IMUCAL;

/**
 * This message contains the integrated solution of the EKF in ECEF.
 * This log includes the inertial data, integrated position, velocity and
 * attitude and status information in one package, instead of using four logs
 * (IMUCOMP, INSRPY, INSPOSXXX and INSVELXXX). The advantage of using this log
 * is that the protocol overhead will be reduced (only one header has to be
 * transmitted together with all contained data).
 */
typedef struct __attribute__((__packed__)) {
  XCOMHeader header;
  float accel[3]; /**< Calibrated acceleration along IMU x-, y-, and z-axis in
                     [m/s2]. */
  float gyro[3];  /**< Calibrated angular rate along IMU x-, y-, and z-axis in
                     [rad/s]. */
  float rpy[3];   /**< Roll, pitch and yaw in [rad]. */
  float vel[3];   /**< Velocity along x-, y- and z-axis in [m/s]. The reference
                     frame can be selected via the PARDAT_VEL parameter (ENU, NED,
                     ECEF or body frame). */
  double pos[2];  /**< Longitude and latitude in [rad]. The selected values are
                     masked in the DATSEL field. */
  float altitude; /**< Altitude/Height in [m]. The content (WGS or MSL or Baro
                     based) can be selected via the PARDAT_POS parameter. The
                     selected value is masked in the DATSEL field. */
  int16_t undulation; /**< Relationship between the geoid and the ellipsoid in
                         [cm]. */
  uint16_t data_selection; /**< Data selection mask. This field contains the
                              selected data sources for this mes- sage  */
  XCOMFooter footer;
} XCOMmsg_INSSOL;

/**
 * This message contains the integrated solution of the EKF in ECEF coordinates.
 */
typedef struct __attribute__((__packed__)) {
  XCOMHeader header;
  float acc[3]; /**< Calibrated acceleration along IMU x-, y-, and z-axis in
                   [m/s2]. */
  float omg[3]; /**< Calibrated angular rate along IMU x-, y-, and z-axis in
                   [rad/s]. */
  double pos_ecef[3]; /**< Estimated position along ECEF x-, y- and z-coordinate
                         in [m] */
  float vel_ecef[3];  /**< Estimated velocity along ECEF x-, y- and z-coordinate
                         in [m/s] */
  float q_nb[4];      /**< Computation frame to NED quaternion */
  XCOMFooter footer;
} XCOMmsg_INSSOLECEF;

/**
 * This message contains the load-factor.
 */
typedef struct __attribute__((__packed__)) {
  XCOMHeader header;
  float load_factor[3]; /**< Load factor x, y and z (unitless) */
  XCOMFooter footer;
} XCOMmsg_LOADFACTOR;

/**
 * This message contains information about the ground speed
 */
typedef struct __attribute__((__packed__)) {
  XCOMHeader header;
  float gnd_speed;   /**< INS speed over ground in [m/s] */
  float track_angle; /**< NS course over ground in [rad] */
  float v_down;      /**< INS down velocity in [m/s] */
  XCOMFooter footer;
} XCOMmsg_INSGNDSPEED;

typedef struct __attribute__((__packed__)) {
  XCOMHeader header;
  float omg_dot[3];
  XCOMFooter footer;
} t_XCOM_OMGDOT;

typedef struct __attribute__((__packed__)) {
  XCOMHeader header;
  float acc[3]; /**< Measured acceleration in ACV (along-track, cross-track,
                   vertical) frame in [m/s2] */
  XCOMFooter footer;
} XCOMmsg_INSTRACKACC;

/**
 * The SYS_STAT message contains the extended system status.
 * The content of this log can be configured via the PARDAT_SYSSTAT parameter.
 */
typedef struct __attribute__((__packed__)) {
  XCOMHeader header;
  uint32_t mode;
  union { /**< System status information */
    XCOMSystemStatus bits;
    uint32_t value;
  } sysstat;
  XCOMFooter footer;
} XCOMmsg_SYSSTAT;

/**
 * The INSRPY message contains the integration filter attitude solution in
 * Euler representation (roll, pitch and yaw). The given Euler angles describe
 * the orientation of the body frame with respect to the navigation frame (NED).
 */
typedef struct __attribute__((__packed__)) {
  XCOMHeader header;
  float rpy[3]; /**< Roll, pitch and yaw in [rad] */
  XCOMFooter footer;
} XCOMmsg_INSRPY;

/**
 * This message contains the integration filter’s orientation solution as
 * direction cosine matrix(DCM)
 */
typedef struct __attribute__((__packed__)) {
  XCOMHeader header;
  float c_nb[9]; /**< Direction Cosine Matrix coefficients */
  XCOMFooter footer;
} XCOMmsg_INSDCM;

/**
 * This message contains the integration filter’s orientation solution.
 * The quaternion provides a redundant, nonsingular attitude representation
 * that is well suited for describing arbitrary and large rotations.
 */
typedef struct __attribute__((__packed__)) {
  XCOMHeader header;
  float q_nb[4]; /**< Computation frame to NED quaternion */
  XCOMFooter footer;
} XCOMmsg_INSQUAT;

/**
 * This message contains the position in WGS84 geodetic coordinates.
 */
typedef struct __attribute__((__packed__)) {
  XCOMHeader header;
  double longitude; /**< Longitude (WGS84) in [rad] */
  double latitude;  /**< Latitude (WGS84) in [rad] */
  float altitude;   /**< Height in [m] */
  XCOMFooter footer;
} XCOMmsg_INSPOSLLH;

/**
 * This message contains the position in UTM coordinates.
 */
typedef struct __attribute__((__packed__)) {
  XCOMHeader header;
  int32_t zone;    /**< UTM zone number */
  double easting;  /**< UTM east position in [m] */
  double northing; /**< UTM north position in [m] */
  float height;    /**< Height in [m] */
  double
      convergence; /**< Meridian convergence for the UTM/UPS projection [rad] */
  XCOMFooter footer;
} XCOMmsg_INSPOSUTM;

/**
 * INS/GNSS position solution in Military Grid Reference System (MGRS).
 */
#define INSMGRS_STRING_LEN 64
typedef struct __attribute__((__packed__)) {
  XCOMHeader header;
  uint32_t error_code;           /**< 0: No error */
  char mgrs[INSMGRS_STRING_LEN]; /**< MGRS coordinate string */
  double
      convergence; /**< Meridian convergence for the UTM/UPS projection [rad] */
  XCOMFooter footer;
} XCOMmsg_INSMGRS;

/**
 * The INSPOSECEF message contains the position in ECEF coordinates.
 */
typedef struct __attribute__((__packed__)) {
  XCOMHeader header;
  double pos_ecef[3]; /**< ECEF x-, y- and z-coordinate in [m] */
  XCOMFooter footer;
} XCOMmsg_INSPOSECEF;

/**
 * This message contains the velocity vector in NED/ENU/ECEF coordinated
 */
typedef struct __attribute__((__packed__)) {
  XCOMHeader header;
  float velocity[3]; /**< INS velocity in [m/s] */
  XCOMFooter footer;
} XCOMmsg_INSVEL;

/**
 * The MAGDATA message contains the magnetometer measurements.
 * The Extended Kalman Filter is able to process magnetic heading as well as the
 * magnetic field vector.
 */
typedef struct __attribute__((__packed__)) {
  XCOMHeader header;
  float
      field[3];  /**< Measured magnetic field along sensors x-, y- and z-axis */
  float heading; /**< Obtained magnetic heading in [rad] */
  float bank;    /**< Magnetometer bank in [rad] */
  float elevation; /**< Magnetometer elevation in [rad] */
  float deviation; /**< Magnetic deviation in [rad] */
  uint32_t status; /**< Magnetometer status */
  XCOMFooter footer;
} XCOMmsg_MAGDATA;

/**
 * The AIRDATA message contains the Micro Air Data Computer (iMADC)
 * measurements. This log is only available with iMAR’s iMADC. The iMADC must be
 * connected via CAN (ARINC825) to the IMS. The maximum data rate of the AIRDATA
 * log is limited to 50 Hz.
 */
typedef struct __attribute__((__packed__)) {
  XCOMHeader header;
  float tas;                   /**< True air speed in [m/s] */
  float ias;                   /**< Indicated air speed in [m/s] */
  float baro_altitude;         /**< Barometric pressure altitude in [m] */
  float baro_altitude_rate;    /**< Barometric altitude rate (rate of climb) in
                                  [m/s] */
  float pd;                    /**< Dynamic pressure in [hPa] */
  float ps;                    /**< Static pressure in [hPa] */
  float oat;                   /**< Outside air temperature in [◦C] */
  float estimated_bias;        /**< Estimated bias in [m] */
  float estimated_scalefactor; /**< Estimated scale factor */
  float estimated_bias_stddev; /**< Estimated bias standard deviation in [m] */
  float estimated_scalefactor_stddev; /**< Estimated scale factor standard
                                         deviation */
  uint32_t status;                    /**< Air Data Computer Status */
  XCOMFooter footer;
} XCOMmsg_AIRDATA;

/**
 * The EKFSTDDEV message contains the standard deviation of the estimated
 * position, velocity, attitude, heading and sensor errors.
 */
typedef struct __attribute__((__packed__)) {
  XCOMHeader header;
  float pos[3];      /**< Standard deviation of estimated easting, northing and
                        height in [m] */
  float vel[3];      /**< Standard deviation of estimated north, east and down
                        velocity in [m/s] */
  float tilt[3];     /**< Standard deviation of error tilt angles around north,
                        east, and down axis. Down axis tilt stddev is equivalent to
                        yaw angle stddev. [rad] */
  float bias_acc[3]; /**< Standard deviation of estimated acceleration bias
                        along IMU x-, y- and z-axis in [m/s2 ] */
  float bias_omg[3]; /**< Standard deviation of estimated angular rate bias
                        along IMU x-, y- and z-axis in [rad/s] */
  float scf_acc[3];  /**< Standard deviation of estimated acceleration scale
                        factor along IMU x-, y- and z-axis */
  float scf_omg[3];  /**< Standard deviation of estimated angular rate scale
                        factor along IMU x-, y- and z- axis */
  float
      scf_odo; /**< Standard deviation of the estimated odometer scale factor */
  XCOMFooter footer;
} XCOMmsg_EKFSTDDEV;

/**
 * The EKFSTDDEV2 message contains the estimated sensor errors as well as the
 * current standard deviations of position parameters and of orientation angles
 * (RPY) by the extended Kalman Filter. Furthermore, it displays the accuracy of
 * other estimated values. In addition to the standard deviations reported in
 * the EKFSTDDEV, estimated sensor nonorthogonality and misalignment standard
 * deviations are included in this log.
 *
 * Further information regarding fields within the EKFSTDDEV2 message:
 *
 * For the standard deviation of the position:
 * The accuracy is given in meters. No special resolution and no special value
 * range are used, this is only limited by the IEEE float format. The float
 * value corresponds to the 1 sigma interval (in meters) supplied by the Kalman
 * Filter.
 *
 * For the standard deviation of the orientation angles (RPY):
 * The float value corresponds to the 1 sigma interval (in rad) supplied by the
 * Kalman Filter. The value range is −π to +π. The resolution is only limited by
 * the float data type. Additionally, there is a pre-defined sequence to follow
 * when using RPY: first, the rotation is around the z-axis, then around the
 * (already rotated) y-axis, finally around the x-axis. If you need further help
 * on this topic, please contact iMAR support (support@imar-navigation.de).
 */
typedef struct __attribute__((__packed__)) {
  XCOMHeader header;
  float pos[3];  /**< Standard deviation of estimated longitude, latitude and
                    height in [m] */
  float vel[3];  /**< Standard deviation of estimated north, east and down
                    velocity in [m/s] */
  float tilt[3]; /**< Standard deviation of estimated tilt around north, east
                    and down axis in [rad] */
  float bias_acc[3]; /**< Standard deviation of estimated acceleration bias
                        along IMU x-, y- and z-axis in [m/s2 ] */
  float bias_omg[3]; /**< Standard deviation of estimated angular rate bias
                        along IMU x-, y- and z-axis in [rad/s] */
  float ma_acc[9];   /**< Standard deviation of estimated acceleration
                        misalignment along IMU x-, y- and z-axis */
  float ma_omg[9];   /**< Standard deviation of estimated angular rate
                        misalignment along IMU x-, y- and z-axis */
  float
      scf_odo; /**< Standard deviation of the estimated odometer scale factor */
  float ma_odo[2]; /**< Standard deviation of estimated odometer misalignment
                      around first and second axis perpendicular to odometer
                      direction */
  XCOMFooter footer;
} XCOMmsg_EKFSTDDEV2;

/**
 * This message contains the estimated sensor errors by the extended Kalman
 * filter in the ECEF frame. In addition to the standard deviations reported in
 * the EKFSTDDEV, estimated sensor nonorthogonality and misalignment standard
 * deviations are included in this log.
 */
typedef struct __attribute__((__packed__)) {
  XCOMHeader header;
  float pos[3]; /**< Standard deviation of estimated ECEF x, y and z in [m] */
  float vel[3]; /**< Standard deviation of estimated ECEF x, y and z velocity in
                   [m/s] */
  float tilt[3]; /**< Standard deviation of estimated tilt around north, east
                    and down axis in [rad] */
  float bias_acc[3]; /**< Standard deviation of estimated acceleration bias
                        along IMU x-, y- and z-axis in [m/s2 ] */
  float bias_omg[3]; /**< Standard deviation of estimated angular rate bias
                        along IMU x-, y- and z-axis in [rad/s] */
  float ma_acc[9];   /**< Standard deviation of estimated acceleration
                        misalignment along IMU x-, y- and z-axis */
  float ma_omg[9];   /**< Standard deviation of estimated angular rate
                        misalignment along IMU x-, y- and z-axis */
  float
      scf_odo; /**< Standard deviation of the estimated odometer scale factor */
  float ma_odo[2]; /**< Standard deviation of estimated odometer misalignment
                      around first and second axis perpendicular to odometer
                      direction */
  XCOMFooter footer;
} XCOMmsg_EKFSTDDEVECEF;

/**
 * This parameter contains the 3x3 position covariance matrix.
 */
typedef struct __attribute__((__packed__)) {
  XCOMHeader header;
  float covar[9]; /**< Position covariance matrix */
  XCOMFooter footer;
} XCOMmsg_EKFPOSCOVAR;

/**
 * The EKFSTATUS message contains the estimated sensor errors by the extended
 * Kalman filter.
 */
typedef struct __attribute__((__packed__)) {
  XCOMHeader header;
  float bias_acc[3]; /**< Estimated bias of acceleration along IMU x-, y- and
                        z-axis in [m/s2] */
  float bias_omg[3]; /**< Estimated bias of angular rate along IMU x-, y- and
                        z-axis in [rad/s] */
  float scf_acc[3]; /**< Estimated scale factor of acceleration along IMU x-, y-
                       and z-axis */
  float scf_omg[3]; /**< Estimated scale factor of angular rate along IMU x-, y-
                       and z-axis */
  float scf_odo;    /**< Estimated odometer scale factor */
  float odo_misalign[2]; /**< Estimated odometer misalignment around first and
                            second axis perpendicular to odometer direction in
                            [rad] */
  XCOMFooter footer;
} XCOMmsg_EKFERROR;

/**
 * The EKFSENSORERR2 message contains the sensor errors estimated by the
 * extended Kalman filter. In addition to the sensor errors reported in the
 * EKFSENSORERR, estimated sensor nonorthogonalities and misalignments are
 * included in this log.
 */
typedef struct __attribute__((__packed__)) {
  XCOMHeader header;
  float bias_acc[3]; /**< Estimated bias of acceleration along IMU x-, y- and
                        z-axis in [m/s2] */
  float bias_omg[3]; /**< Estimated bias of angular rate along IMU x-, y- and
                        z-axis in [rad/s] */
  float ma_acc[9];   /**< Accelerometer misalignment matrix */
  float ma_omg[9];   /**< Gyro misalignment matrix */
  float scf_odo;     /**< Estimated odometer scale factor */
  float ma_odo[2]; /**< Estimated odometer misalignment around first and second
                      axis perpendicular to odometer direction in [rad] */
  XCOMFooter footer;
} XCOMmsg_EKFERROR2;

typedef struct __attribute__((__packed__)) {
  XCOMHeader header;
  double a_bib[3];
  double w_bib[3];
  double dt;
  double delta_pos_ned[3]; /**< position in north/east/down in m. */
  double v_ned[3];         /**< velocity in north/east/down in m/s. */
  double C_nb[9];          /**< rotation matrix from body->nav-frame */
  uint32_t time_index; /**< current time index (start value: 0 / new sensor data
                          received: timeIndex++) */
  uint8_t is_zupt;
  uint8_t reserved[3];
  XCOMFooter footer;
} XCOMmsg_STEPDETECT;

typedef struct __attribute__((__packed__)) {
  XCOMHeader header;

  uint8_t ucSatsAvailablePSR;
  uint8_t ucSatsUsedPSR;
  uint8_t ucSatsAvailableRR;
  uint8_t ucSatsUsedRR;

  uint8_t ucSatsAvailableTDCP;
  uint8_t ucSatsUsedTDCP;
  uint8_t ucRefSatTDCP;
  uint8_t usReserved;

  uint32_t uiUsedSatsPSR_GPS;
  uint32_t uiOutlierSatsPSR_GPS;

  uint32_t uiUsedSatsPSR_GLONASS;
  uint32_t uiOutlierSatsPSR_GLONASS;

  uint32_t uiUsedSatsRR_GPS;
  uint32_t uiOutlierSatsRR_GPS;

  uint32_t uiUsedSatsRR_GLONASS;
  uint32_t uiOutlierSatsRR_GLONASS;

  uint32_t uiUsedSatsTDCP_GPS;
  uint32_t uiOutlierSatsTDCP_GPS;

  uint32_t uiUsedSatsTDCP_GLONASS;
  uint32_t uiOutlierSatsTDCP_GLONASS;
  XCOMFooter footer;
} t_XCOM_EKFTIGHTLY;

typedef struct __attribute__((__packed__)) {
  XCOMHeader header;
  float power[32];
  XCOMFooter footer;
} t_XCOM_POWER;

typedef struct __attribute__((__packed__)) {
  XCOMHeader header;
  float temperature[XCOM_MAX_NUMBER_OF_TEMPERATURES];
  XCOMFooter footer;
} t_XCOM_TEMP;

typedef struct __attribute__((__packed__)) {
  XCOMHeader header;
  uint32_t uiRRidx[4];
  uint32_t uiRRvalue[4];
  XCOMFooter footer;
} t_XCOM_ADC24STATUS;

typedef struct __attribute__((__packed__)) {
  XCOMHeader header;
  int32_t acc[3];
  uint16_t frameCounter;
  int16_t temperature;
  uint8_t errorStatus;
  uint8_t intervalCounter[3];
  XCOMFooter footer;
} t_XCOM_ADC24DATA;

typedef struct __attribute__((__packed__)) {
  XCOMHeader header;
  double posECEF[3];
  double velECEF[3];
  double q_nb[4];
  float acc[3];
  float omg[3];
  float posStdDev[3];
  float velStdDev[3];
  float attStdDev[3];
  XCOMFooter footer;
} t_XCOM_MVCSLAVE;

typedef struct __attribute__((__packed__)) {
  double gpsTime;
  double posECEF[3];
  double velECEF[3];
  double q_nb[4];
  float acc[3];
  float omg[3];
  float posStdDev[3];
  float velStdDev[3];
  float attStdDev[3];
  uint16_t globalStatus;
  uint16_t reserved;
} t_XCOM_MVCdataType;

typedef struct __attribute__((__packed__)) {
  XCOMHeader header;
  int slaveIdx;
  t_XCOM_MVCdataType master;
  t_XCOM_MVCdataType slave;
  XCOMFooter footer;
} t_XCOM_MVCDATA;

typedef struct __attribute__((__packed__)) {
  XCOMHeader header;
  uint32_t status;
  uint32_t alarm;
  char serialNum[32];
  uint32_t mode;
  uint32_t contrast;
  float laserCurrent;
  float tcx0;
  float heatP;
  float sig;
  float temperature;
  int32_t steer;
  float atune;
  int32_t phase;
  uint32_t discOk;
  uint32_t timeSincePowerOn;
  uint32_t timeSinceLock;
  uint8_t dataValid;
  uint8_t reserved;
  uint16_t fwStatus;
  XCOMFooter footer;
} t_XCOM_CSACDATA;

enum GnssSolStatus {
  SOL_COMPUTED = 0,     /**< Solution computed */
  INSUFFICIENT_OBS = 1, /**< Insufficient observations */
  NO_CONVERGENCE = 2,   /**< No convergence */
  SINGULARITY = 3,      /**< Singularity at parameters matrix */
  COV_TRACE = 4,        /**< Covariance trace exceeds maximum (trace >1000 m) */
  TEST_DIST = 5,        /**< Test distance exceeded (maximum of 3 rejections if
                           distance >10 km) */
  COLD_START = 6,       /**< Not yet converged from cold start */
  V_H_LIMIT = 7,     /**< Height or velocity limits exceeded (in accordance with
                        export licensing restrictions) */
  VARIANCE = 8,      /**< Variance exceeds limits */
  RESIDUALS = 9,     /**< Residuals are too large */
  DELTA_POS = 10,    /**< Delta position is too large */
  NEGATIVE_VAR = 11, /**< Negative variance */
  RESERVED = 12,     /**< Reserved for further use */
  INTEGRITY_WARNING = 13, /**< Large residuals make position unreliable */
  PENDING =
      18, /**< When a FIX position command is entered, the receiver computes its
             own position and determines if the fixed position is valid */
  INVALID_FIX = 19,  /**< The fixed position, entered using the FIX position
                        command, is not valid */
  UNAUTHORIZED = 20, /**< Position type is unauthorized - HP or XP on a receiver
                        not authorized for it */
  ANTENNA_WARNING = 21 /**< Antenna warning */
};

enum GnssPosVelType {
  NONE = 0,     /**< No solution */
  FIXEDPOS = 1, /**< Position has been fixed by the FIX POSITION command */
  FIXEDHEIGHT =
      2, /**< Position has been fixed by the FIX HEIGHT/AUTO command */
  DOPPLER_VELOCITY = 8, /**< Velocity computed using instantaneous Doppler */
  SINGLE = 16,          /**< Single point position */
  PSRDIFF = 17,         /**< Pseudorange differential solution */
  SBAS = 18, /**< Solution calculated using corrections from an SBAS */
  PROPAGATED =
      19,        /**< Propagated by a Kalman filter without new observations */
  OMNISTAR = 20, /**< OmniSTAR VBS position (L1 sub-metre) */
  L1_FLOAT = 32, /**< Floating L1 ambiguity solution */
  IONOFREE_FLOAT = 33, /**< Floating ionospheric-free ambiguity solution */
  NARROW_FLOAT = 34,   /**< Floating narrow-lane ambiguity solution */
  L1_INT = 48,         /**< Integer L1 ambiguity solution */
  WIDE_INT = 49,       /**< Integer wide-lane ambiguity solution */
  NARROW_INT = 50,     /**< Integer narrow-lane ambiguity solution */
  OMNISTAR_HP = 64,    /**< OmniSTAR HP position */
  OMNISTAR_XP = 65,    /**< OmniSTAR XP or OmniSTAR G2 (GPS+GLONASS) position */
  PPP_CONVERGING = 68, /**< Converging TerraStar-C solution */
  PPP = 69             /**< Converged TerraStar-C solution */
};

/**
 * The GNSSSOL message contains the GNSS solution.
 * If the INS is aided with RTK corrections, this log contains the RTK position.
 */
typedef struct __attribute__((__packed__)) {
  XCOMHeader header;
  double lon; /**< GNSS longitude (WGS84) in [rad] */
  double lat; /**< GNSS latitude (WGS84) in [rad] */
  float alt;  /**< GNSS altitude (WGS84 MSL) in [m]. Vertical distance above the
                 geoid. */
  float undulation; /**< Undulation - the relationship between the geoid and the
                       ellipsoid in [m] */
  float v_ned[3];   /**< GNSS north, east and down velocity in [m/s] */
  float stddev_pos[3];  /**< Standard deviation of GNSS longitude, latitude and
                           altitude in [m] */
  float stddev_vel[3];  /**< Standard deviation of GNSS north, east and down
                           velocity in [m/s] */
  uint16_t sol_status;  /**< GNSS Solution status */
  uint16_t posvel_type; /**< GNSS position/velocity type */
  float pdop; /**< Position Dilution Of Precision - uncertainty indicator for 3D
                 parameters (latitude, longitude, height) */
  uint8_t sats_used;    /**< Number of satellites used in GNSS solution */
  uint8_t sats_tracked; /**< Number of satellites tracked */
  uint16_t station_id;  /**< Reference station ID of differential corrections */
  float diff_age;       /**< Differential age in [sec] - differential correction
                           indicator */
  float sol_age;        /**< Solution age in [sec] */
  uint32_t gnss_status; /**< GNSS receiver status */
  XCOMFooter footer;
} XCOMmsg_GNSSSOL;

typedef struct __attribute__((__packed__)) {
  XCOMHeader header;
  double lon; /**< GNSS longitude (WGS84) in [rad] */
  double lat; /**< GNSS latitude (WGS84) in [rad] */
  float alt;  /**< GNSS altitude (WGS84 MSL) in [m]. Vertical distance above the
                 geoid */
  float undulation; /**< Undulation - the relationship between the geoid and the
                       ellipsoid in [m] */
  float pos_stddev[3];   /**< Standard deviation of GNSS longitude, latitude and
                            altitude in [m] */
  float vel_ned[3];      /**< GNSS north, east and down velocity in [m/s] */
  float vel_stddev[3];   /**< Standard deviation of GNSS north, east and down
                            velocity in [m/s] */
  float displacement[3]; /**< Displacement in NED frame in [m] */
  float displacement_stddev[3]; /**< Standard deviation of displacement in NED
                                   frame in [m] */
  uint16_t solution_status;     /**< GNSS Solution status */
  float dop[2]; /**< Position Dilution Of Precision - uncertainty indicator for
                   3D parameters (latitude, longitude, height) */
  uint8_t sats_pos;         /**< Satellites used for position solution */
  uint8_t sats_vel;         /**< Satellites used for velocity solution */
  uint8_t sats_displacment; /**< Satellites used for displacement solution */
  uint16_t reserved;        /**< Reserved for further use */
  float diff_age;       /**< Differential age in [sec] - differential correction
                           indicator */
  float solution_age;   /**< Solution age in [sec] */
  uint32_t gnss_status; /**< GNSS receiver status */
  XCOMFooter footer;
} XCOMmsg_GNSSSOLCUST;

/**
 * This message contains the GNSS position in ECEF coordinates
 */
typedef struct __attribute__((__packed__)) {
  XCOMHeader header;
  double pos[3];   /**< GNSS position in ECEF coordinates [m] */
  float stddev[3]; /**< Standard deviation of GNSS position solution [m] */
  uint16_t solution_status; /**< GNSS Solution status */
  uint16_t position_type;   /**< GNSS position type */
  float pdop; /**< Position Dilution Of Precision - uncertainty indicator for 3D
                 parameters (latitude, longitude, height) */
  uint8_t sats_used;    /**< Number of satellites used in GNSS solution */
  uint8_t sats_tracked; /**< Number of satellites tracked */
  uint16_t station_id;  /**< Reference station ID of differential corrections */
  float diff_age;       /**< Differential age in [sec] - differential correction
                           indicator */
  float solution_age;   /**< Solution age in [sec] */
  uint32_t gnss_status; /**< GNSS receiver status */
  XCOMFooter footer;
} XCOMmsg_GNSSPOSECEF;

/**
 * This message contains the GNSS velocity in ECEF coordinates
 */
typedef struct __attribute__((__packed__)) {
  XCOMHeader header;
  double vel[3];   /**< GNSS velocity in ECEF coordinates [m] */
  float stddev[3]; /**< Standard deviation of GNSS velocity solution [m] */
  uint16_t solution_status; /**< GNSS Solution status */
  uint16_t velocity_type;   /**< GNSS velocity type */
  float pdop; /**< Position Dilution Of Precision - uncertainty indicator for 3D
                 parameters (latitude, longitude, height) */
  uint8_t sats_used;    /**< Number of satellites used in GNSS solution */
  uint8_t sats_tracked; /**< Number of satellites tracked */
  uint16_t station_id;  /**< Reference station ID of differential corrections */
  float diff_age;       /**< Differential age in [sec] - differential correction
                           indicator */
  float solution_age;   /**< Solution age in [sec] */
  uint32_t gnss_status; /**< GNSS receiver status */
  XCOMFooter footer;
} XCOMmsg_GNSSVELECEF;

/*
 * This message contains satellite specific information.
 */
typedef struct __attribute__((__packed__)) {
  XCOMHeader header;
  uint32_t sv_id; /**< Space Vehicle ID. SVID 1-32 are GPS, 38-70 are GLONASS */
  double pos_ecef[3]; /**< Satellite position for x-, y- and z- axis within ECEF
                         frame in [m] */
  double vel_ecef[3]; /**< Satellite velocity for x-, y- and z- axis within ECEF
                         frame in [m/s] */
  float cn0[3]; /**< Satellite carrier to noise ratio in dB-Hz for L1, L2 and L5
                   carrier */
  float clock_error; /**< Error caused by SV clock */
  float iono_error;  /**< Error caused by ionospheric effects */
  float tropo_error; /**< Error caused by tropospheric effects */
  float elevation;   /**< Satellite elevation angle in [rad] */
  float azimuth;     /**< Satellite azimuth angle in [rad] */
  XCOMFooter footer;
} XCOMmsg_GNSSSATINFO;

typedef struct __attribute__((__packed__)) {

  XCOMHeader header;
  struct {
    uint32_t sat_system; /**< GNSS satellite system identifier */
    uint16_t
        sat_id; /**< Space Vehicle ID. SVID 1-32 are GPS, 38-70 are GLONASS */
    int16_t glo_freq; /**< GLONASS frequency */
    float cn0; /**< Satellite carrier to noise ratio in dB-Hz for L1, L2 and L5
                  carrier */
    uint32_t reject; /**< Range reject code from pseudorange filter */
    float azimuth;   /**< Satellite azimuth angle in [rad] */
    float elevation; /**< Satellite elevation angle in [rad] */
  } INF[XCOM_MAX_SATCHANNELS];
  XCOMFooter footer;
} XCOMmsg_SATINF;

#define GNSSTIME_STATUS_INVALID 0 /**< Status is invalid */
#define GNSSTIME_STATUS_VALID 1   /**< Status is valid */
#define GNSSTIME_STATUS_WARNING                                                \
  2 /**< Indicates that the leap seconds value is used as a default due to the \
       lack of an almanac */
/**
 * This message provides several time related pieces of information
 * including receiver clock offset and UTC time and offset.
 */
typedef struct __attribute__((__packed__)) {
  XCOMHeader header;
  double
      utc_offset; /**< The offset of GPS reference time from UTC time, computed
                     using almanac parameters. UTC time is GPS reference time
                     plus the current UTC offset plus the receiver clock offset:
                     UTC time = GPS reference time + offset + UTC offset */
  double
      offset; /**< Receiver clock offset, in seconds from GPS reference time. A
                 positive offset implies that the receiver clock is ahead of GPS
                 reference time. To derive GPS reference time, use the following
                 formula: GPS reference time = receiver time - offset */
  uint32_t utc_year; /**< UTC year */
  uint8_t utc_month; /**< UTC month (0-12). If UTC time is unknown, the value is
                        0 */
  uint8_t
      utc_day; /**< UTC day (0-31). If UTC time is unknown, the value is 0 */
  uint8_t
      utc_hour; /**< UTC hour (0-23). If UTC time is unknown, the value is 0 */
  uint8_t
      utc_min; /**< UTC min (0-59). If UTC time is unknown, the value is 0 */
  uint32_t utc_ms; /**< UTC milliseconds (0- 60999). Maximum of 60999 when leap
                      second is applied. */
  uint32_t utc_status; /**< UTC Status */
  XCOMFooter footer;
} XCOMmsg_GNSSTIME;

/**
 * This message contains the dual antenna GNSS heading information.
 */
typedef struct __attribute__((__packed__)) {
  XCOMHeader header;
  float heading;            /**< Heading in [rad] */
  float heading_stddev;     /**< Heading standard deviation in [rad] */
  float pitch;              /**< Pitch angle in [rad] */
  float pitch_stddev;       /**< Pitch standard deviation in [rad] */
  uint16_t solution_status; /**< GNSS solution status */
  uint16_t position_type;   /**< GNSS position type */
  uint16_t reserved;        /**< Reserved for further use */
  uint8_t sats_used; /**< Number of satellites used in dual-antenna solution */
  uint8_t sats_tracked; /**< Number of satellites tracked */
  uint32_t gnss_status; /**< GNSS receiver status */
  XCOMFooter footer;
} XCOMmsg_GNSSHDG;

/**
 * This message contains the GNSS antenna lever arm in body frame being
 * estimated by the EKF.
 */
typedef struct __attribute__((__packed__)) {
  XCOMHeader header;
  float primary_ant_offset[3]; /**< Estimated x, y and z primary antenna offset
                                  in IMU frame in [m] */
  float primary_ant_offset_stddev[3]; /**< Primary antenna standard deviation of
                                         estimated x, y and z offset in [m] */
  float secondary_ant_offset[3]; /**< Estimated x, y and z secondary antenna
                                    offset in IMU frame in [m] */
  float secondary_ant_offset_stddev[3]; /**< Secondary antenna standard
                                           deviation of estimated x, y and z
                                           offset in [m] */
  XCOMFooter footer;
} XCOMmsg_GNSSLEVERARM;

/**
 * This message contains DOP values for the satellites used in the GNSS solution
 */
typedef struct __attribute__((__packed__)) {
  XCOMHeader header;
  float pdop;  /**< Position dilution of precision - assumes 3D position is
                  unknown and receiver clock offset is known */
  float hdop;  /**< Horizontal dilution of precision. */
  float vdop;  /**< Vertical dilution of precision */
  float gdop;  /**< Geometric dilution of precision - assumes 3D position and
                  receiver clock offset (all 4 parameters) are unknown */
  float htdop; /**< Horizontal position and time dilution of precision. */
  float tdop; /**< Time dilution of precision - assumes 3D position is known and
                 only the receiver clock offset is unknown */
  XCOMFooter footer;
} XCOMmsg_GNSSDOP;

/**
 * This message log contains information regarding the GNSS voter module.
 */
typedef struct __attribute__((__packed__)) {
  XCOMHeader header;
  uint8_t sats_used_int; /**< Number of satellites used for internal receiver */
  uint8_t sats_used_ext; /**< Number of satellites used for external receiver */
  uint16_t reserved;     /**< Reserved for further use */
  float stddev_hdg_int;  /**< Standard deviation heading of internal receiver in
                            [rad] */
  float stddev_hdg_ext;  /**< Standard deviation heading of external receiver in
                            [rad] */
  float stddev_pos_int; /**< Standard deviation position of internal receiver in
                           [m] */
  float stddev_pos_ext; /**< Standard deviation position of external receiver in
                           [m] */
  uint32_t status;      /**< GNSS Voter Status Mask, */
  XCOMFooter footer;
} XCOMmsg_GNSSVOTER;

/**
 * This message contains the RTK quality ENU baselines.
 */
typedef struct __attribute__((__packed__)) {
  XCOMHeader header;
  double enu[3];       /**< GNSS baseline in east, north and up direction [m] */
  float stddev_enu[3]; /**< Standard deviation of ENU baseline [m] */
  uint16_t solution_status; /**< GNSS solution status */
  uint16_t position_type;   /**< GNSS position type */
  uint8_t sats_tracked;     /**< Number of satellites tracked */
  uint8_t sats_used;        /**< Number of satellites used in solution */
  uint8_t extended_solution_status; /**< Extended GNSS solution status */
  uint8_t reserved;                 /**< Reserved for further use */
  XCOMFooter footer;
} XCOMmsg_GNSSALIGNBSL;

/**
 * This message contains the hardware monitor status information of the GNSS
 * receiver (temperature, antenna current and voltages). Each item consists of a
 * description, a related status and a float value.
 */
typedef struct __attribute__((__packed__)) {
  XCOMHeader header;
  struct {
    float val;       /**< Value related to the status information */
    uint32_t status; /**< The related status */
  } GnssHwMonitor[16];
  XCOMFooter footer;
} XCOMmsg_GNSSHWMON;

#define XCOMMSG_PORTSTATS_MAXPORTS 3 /**< Number of supported ports */
/**
 * This log contains port statistics of the integrated GNSS receiver
 * COM ports and may be used to debug issues with e.g. correction data input.
 */
typedef struct __attribute__((__packed__)) {
  XCOMHeader header;
  int number_of_elements;
  struct {
    uint32_t port; /**< COM port */
    uint32_t
        rx_chars; /**< Total number of characters received through this port */
    uint32_t tx_chars; /**< Total number of characters transmitted through this
                          port */
    uint32_t acc_rx_chars; /**< Total number of accepted characters received
                              through this port */
    uint32_t droppped_rx_chars; /**< Number of software overruns in receive */
    uint32_t interrupts;        /**< Number of interrupts on this port */
    uint32_t breaks;            /**< Number of breaks (only for serial ports) */
    uint32_t
        parity_errors; /**< Number of parity errors (only for serial ports) */
    uint32_t
        frame_errors; /**< Number of framing errors (only for serial ports) */
    uint32_t rx_overrun; /**< Number of hardware overruns in receive */
  } stats[XCOMMSG_PORTSTATS_MAXPORTS];
  XCOMFooter footer;
} XCOMmsg_PORTSTATS;

typedef struct __attribute__((__packed__)) {
  XCOMHeader header;

  /* Debug */
  double StatFiltPos;
  double AppliedFreqHz;
  double AppliedAmplMeter;
  double AppliedSigWaveHeightMeter;
  double PZpos;
  double ZDpos;
  double ZDvel;
  double AccZnavDown;

  /* Data */
  double HeavePosVelDown[2];
  uint32_t HeaveAlgoStatus1;
  uint32_t HeaveAlgoStatus2;

  XCOMFooter footer;
} XCOMmsg_HEAVE;

typedef struct __attribute__((__packed__)) {
  XCOMHeader header;
  float speed;
  int32_t ticks;
  XCOMFooter footer;
} XCOMmsg_WHEELDATA;

typedef struct __attribute__((__packed__)) {
  XCOMHeader header;

  /* Inertial */
  float accel[3];
  float omg[3];
  float delta_theta[12];
  float delta_velocity[12];

  /* Navigation */
  double q_nb[4];
  double pos_llh[3];
  double vel_ned[3];

  /* Status */
  uint32_t sys_stat;
  uint32_t ekf_stat_low;
  uint32_t ekf_stat_hi;

  /* Odometer */
  float odo_speed;
  int32_t odo_ticks;
  uint32_t odo_interval;
  uint32_t odo_trig_event;
  uint32_t odo_trig_next_event;

  XCOMFooter footer;
} XCOMmsg_POSTPROC;

typedef struct __attribute__((__packed__)) {
  XCOMHeader header;
  uint32_t uiErrorMask;
  uint8_t ucControllerStatus;
  uint8_t ucTransceiverStatus;
  uint8_t ucProtocolStatus;
  uint8_t ucProtocolLocation;
  XCOMFooter footer;
} XCOMmsg_CANSTATUS;

typedef struct __attribute__((__packed__)) {
  XCOMHeader header;
  double time_event[2];
  XCOMFooter footer;
} XCOMmsg_EVENTTIME;

/**
 * ******************************************************************************************
 * XCOM command definition
 * ******************************************************************************************
 */
typedef struct __attribute__((__packed__)) {
  XCOMHeader header;
  uint16_t cmd_id;
  uint16_t specific;
} XCOMCmdHeader;

typedef struct __attribute__((__packed__)) {
  XCOMCmdHeader header;
  uint8_t msg_id;
  uint8_t trigger;
  uint16_t cmd_par;
  uint16_t divider;
  XCOMFooter footer;
} XCOMCmd_LOG;

typedef struct __attribute__((__packed__)) {
  XCOMCmdHeader header;
  uint16_t state;
  uint16_t channel;
  XCOMFooter footer;
} XCOMCmd_XCOM;

typedef struct __attribute__((__packed__)) {
  XCOMCmdHeader header;
  uint32_t conf_cmd;
  XCOMFooter footer;
} XCOMCmd_CONF;

typedef struct __attribute__((__packed__)) {
  XCOMCmdHeader header;
  uint16_t fpga_cmd;
  uint16_t param;
  XCOMFooter footer;
} XCOMCmd_FPGA;

typedef struct __attribute__((__packed__)) {
  XCOMCmdHeader header;
  uint16_t ekf_cmd;
  uint16_t number_of_param;
  float *param;
  XCOMFooter footer;
} XCOMCmd_EKF;

/**
 * ******************************************************************************************
 * XCOM parameter definition
 * ******************************************************************************************
 */
typedef struct __attribute__((__packed__)) {
  uint16_t param_id;  /**< Parameter ID */
  uint8_t reserved;   /** Reserved for further use */
  uint8_t is_request; /** Requesting/Changing a parameter */
} XCOMParHeader;

typedef struct __attribute__((__packed__)) {
  XCOMHeader header;
  XCOMParHeader param_header;
  uint8_t payload[32];
  XCOMFooter footer;
} XCOMPar_Char32;

typedef struct __attribute__((__packed__)) {
  XCOMHeader header;
  XCOMParHeader param_header;
  uint8_t payload[64];
  XCOMFooter footer;
} XCOMPar_Char64;

typedef struct __attribute__((__packed__)) {
  XCOMHeader header;
  XCOMParHeader param_header;
  uint16_t password;
  uint16_t reserved1;
  uint8_t payload[32];
  XCOMFooter footer;
} XCOMParSYS_CALDATE;

/*
 * This parameter is read-only.
 * The parameter contains the system’s current uptime (time since last reboot)
 * in [sec].
 */
typedef struct __attribute__((__packed__)) {
  XCOMHeader header;
  XCOMParHeader param_header;
  float up_time; /**< Current uptime of the system in [sec] */
  XCOMFooter footer;
} XCOMParSYS_UPTIME;

typedef struct __attribute__((__packed__)) {
  XCOMHeader header;
  XCOMParHeader param_header;
  uint32_t operation_hour_counter;
  XCOMFooter footer;
} XCOMParSYS_OPERATIONHOUR;

typedef struct __attribute__((__packed__)) {
  XCOMHeader header;
  XCOMParHeader param_header;
  uint16_t main_timing;
  uint16_t password;
  XCOMFooter footer;
} XCOMParSYS_MAINTIMING;

/*
 * This parameter defines the time synchronization source
 */
typedef struct __attribute__((__packed__)) {
  XCOMHeader header;
  XCOMParHeader param_header;
  uint16_t mode;
  uint16_t reserved1;
  XCOMFooter footer;
} XCOMParSYS_SYNCMODE;

typedef struct __attribute__((__packed__)) {
  XCOMHeader header;
  XCOMParHeader param_header;
  uint16_t prescaler;
  uint16_t password;
  XCOMFooter footer;
} XCOMParSYS_PRESCALER;

typedef struct __attribute__((__packed__)) {
  XCOMHeader header;
  XCOMParHeader param_header;
  uint32_t boot_mode;
  XCOMFooter footer;
} XCOMParSYS_BOOTMODE;

typedef struct __attribute__((__packed__)) {
  XCOMHeader header;
  XCOMParHeader param_header;
  uint32_t calib_id;
  XCOMFooter footer;
} XCOMParSYS_CALIBID;

typedef struct __attribute__((__packed__)) {
  XCOMHeader header;
  XCOMParHeader param_header;
  uint8_t major;
  uint8_t minor;
  uint16_t imu_type;
  XCOMFooter footer;
} XCOMParSYS_FPGAVERSION;

/*
 * This read-only parameter holds the checksums calculated over the
 * internal configuration binary files.
 */
typedef struct __attribute__((__packed__)) {
  XCOMHeader header;
  XCOMParHeader param_header;
  uint16_t rom_crc; /**< CRC16 calculated over ROM content */
  uint16_t ram_crc; /**< CRC16 calculated over RAM content */
  XCOMFooter footer;
} XCOMParSYS_CONFIGCRC;

typedef struct __attribute__((__packed__)) {
  XCOMHeader header;
  XCOMParHeader param_header;
  uint32_t imu_type;
  uint32_t gnss_type;
  uint32_t hw_type;
  int8_t mac_addr[16]; /**< read-only: This parameter can not be changed */
  int8_t cpu_serial_number[16]; /**< read-only: This parameter can not be
                                   changed */
  int8_t
      cpu_order_code[64]; /**< read-only: This parameter can not be changed */
  XCOMFooter footer;
} XCOMParSYS_EEPROM;

typedef struct __attribute__((__packed__)) {
  XCOMHeader header;
  XCOMParHeader param_header;
  float xyz[3];
  XCOMFooter footer;
} XCOMParIMU_MISALIGN;

typedef struct __attribute__((__packed__)) {
  XCOMHeader header;
  XCOMParHeader param_header;
  uint32_t type;
  XCOMFooter footer;
} XCOMParIMU_TYPE;

/**
 * This parameter adjusts the timestamp of the inertial data
 * which is used to synchronize IMU measurements with external sensors.
 * It can be used to correct for known and constant group or transmission
 * delays.
 */
typedef struct __attribute__((__packed__)) {
  XCOMHeader header;
  XCOMParHeader param_header;
  double latency; /**< IMU latency in [sec] */
  XCOMFooter footer;
} XCOMParIMU_LATENCY;

typedef struct __attribute__((__packed__)) {
  XCOMHeader header;
  XCOMParHeader param_header;
  float scf_acc[3];
  float bias_acc[3];
  float scf_omg[3];
  float bias_omg[3];
  XCOMFooter footer;
} XCOMParIMU_CALIB;

typedef struct __attribute__((__packed__)) {
  XCOMHeader header;
  XCOMParHeader param_header;

  /* Idx. 0-2: 1. Col
   * Idx. 3-5: 2. Col
   * Idx. 6-8: 3. Col
   */
  double cc_acc[9];
  double cc_omg[9];

  XCOMFooter footer;
} XCOMParIMU_CROSSCOUPLING;

typedef struct __attribute__((__packed__)) {
  XCOMHeader header;
  XCOMParHeader param_header;
  double refpoint_offset[3];
  XCOMFooter footer;
} XCOMParIMU_REFPOINTOFFSET;

/**
 * This parameter configures the bandstop filter which can be enabled for the
 * accelerations and angular rates which are contained in the INSSOL log. This
 * can be used to remove sinusoidal noise due to e.g. gyro dithering from the
 * measured inertial data. The default values are set so this apparent noise is
 * filtered out. Output in the INSRPY log has to be enabled using the PARDAT_IMU
 * parameter.
 */
typedef struct __attribute__((__packed__)) {
  XCOMHeader header;
  XCOMParHeader param_header;
  float bandwidth; /**< Bandwidth of the bandstop in [Hz] */
  float center;    /**< Center frequency of the bandstop in [Hz] */
  XCOMFooter footer;
} XCOMParIMU_BANDSTOP;

/*
 * This parameter configures the downsampling behavior prior to strapdown
 * calculations if the prescaler value in PARSYS_PRESCALER is greater than 1.
 */
#define PARIMU_COMPMETHOD_SNAPSHOT 0
#define PARIMU_COMPMETHOD_AVERAGING 1
#define PARIMU_COMPMETHOD_CONING 2
typedef struct __attribute__((__packed__)) {
  XCOMHeader header;
  XCOMParHeader param_header;
  uint32_t method; /**< Method used to downsample:
                                                    0 = snapshot (module
                      diabled) 1 = averaging 2 = coning and sculling
                                    */
  XCOMFooter footer;
} XCOMParIMU_COMPMETHOD;

/**
 * This parameter configures coordinates of the accelerometers in the body
 * frame. These values are used to compensate for the size effect.
 */
typedef struct __attribute__((__packed__)) {
  XCOMHeader header;
  XCOMParHeader param_header;
  double r_acc[9];
  XCOMFooter footer;
} XCOMParIMU_ACCLEVERARM;

typedef struct __attribute__((__packed__)) {
  XCOMHeader header;
  XCOMParHeader param_header;
  uint8_t is_increment;
  uint8_t delta_v_frame;
  uint8_t n_delta_theta;
  uint8_t use_secondary_accel_sensor;
  XCOMFooter footer;
} XCOMParIMU_STRAPDOWNCONF;

typedef struct __attribute__((__packed__)) {
  XCOMHeader header;
  XCOMParHeader param_header;
  float range_accel; /**< [m/s/s] */
  float range_gyro;  /**< [rad/s] */
  XCOMFooter footer;
} XCOMParIMU_RANGE;

/**
 * This parameter defines the rotation matrix from the iNAT standard
 * enclosure frame (z down, x axis pointing from the plugs into the device)
 * to a custom enclosure frame (e.g. z up, x axis pointing from the plugs into
 * the device). This matrix is applied after IMU calibration and before using
 * the data in strapdown calculations and all output logs.
 */
typedef struct __attribute__((__packed__)) {
  XCOMHeader header;
  XCOMParHeader param_header;
  double c_enc_imu[9];
  XCOMFooter footer;
} XCOMParIMU_C_ENC_IMU;

/**
 * This parameter is read-only and will be factory set during production
 */
typedef struct __attribute__((__packed__)) {
  XCOMHeader header;
  XCOMParHeader param_header;
  uint8_t port; /**< Internal com port (hw dependent) */
  uint8_t reserved2;
  uint16_t password;
  XCOMFooter footer;
} XCOMParGNSS_PORT;

typedef struct __attribute__((__packed__)) {
  XCOMHeader header;
  XCOMParHeader param_header;
  char file_name[256];
  XCOMFooter footer;
} XCOMParGNSS_FWUPDATE;

/**
 * This parameter is read-only and will be set during production
 */
typedef struct __attribute__((__packed__)) {
  XCOMHeader header;
  XCOMParHeader param_header;
  uint32_t baud; /**< Baud rate of internal GNSS connection */
  uint8_t reserved2;
  uint16_t password;
  XCOMFooter footer;
} XCOMParGNSS_BAUD;

/**
 * This parameter enables or disables GNSS dual antenna mode inside the GNSS
 * module.
 */
#define GNSS_DUALANTMODE_DISABLE 0
#define GNSS_DUALANTMODE_ENABLE 1
typedef struct __attribute__((__packed__)) {
  XCOMHeader header;
  XCOMParHeader param_header;
  uint32_t mode; /**< GNSS dual antenna mode */
  XCOMFooter footer;
} XCOMParGNSS_DUALANTMODE;

/*
 * This parameter configures the GNSS lever arm of the primary or secondary
 * antenna. The measurement should be done as accurately as possible and the
 * standard deviations should over bound the actual error to be expected from
 * the measurement. The antenna offset is the vector from the enclosure
 * reference point to the antenna phase center in INS enclosure frame
 * coordinates. For every standard deviation, only values > 0 are accepted.
 */
#define GNSS_ANTOFFSET_SEL_PRIM_INT 0 /**< Primary internal antenna */
#define GNSS_ANTOFFSET_SEL_SEC_INT 1  /**< Secondary internal antenna */
#define GNSS_ANTOFFSET_SEL_PRIM_EXT                                            \
  2 /**< Primary external antenna (optional) */
#define GNSS_ANTOFFSET_SEL_SEC_EXT                                             \
  3 /**< Secondary external antenna (optional) */
typedef struct __attribute__((__packed__)) {
  XCOMHeader header;
  uint16_t param_id;
  uint8_t rx_selection; /**< Selection between primary and secondary antenna */
  uint8_t is_request;
  float lever_arm[3]; /**< Distance between IMU’s reference point of measurement
                         and GNSS antenna in x-, y- and z-direction in [m] */
  float stddev[3];    /**< Uncertainty of the measured lever arm in x-, y- and
                         z-direction in [m] */
  XCOMFooter footer;
} XCOMParGNSS_ANTOFFSET;

typedef struct __attribute__((__packed__)) {
  XCOMHeader header;
  XCOMParHeader param_header;
  uint8_t udp_enable;
  uint8_t reserved1[3];
  uint32_t udp_addr;
  uint32_t udp_port;
  uint32_t tcp_port;
  XCOMFooter footer;
} XCOMParGNSS_GATEWAYCFG;

/**
 * This parameter enables/disables the RTK mode.
 * If RTKMODE is set to Auto-Detection, the systems will check the GNSS
 * receiver’s model during configuration phase. It is recommended to use the
 * Auto-Detection mode.
 */
#define GNSS_RTKMODE_MODESELECTION_AUTO 2
typedef struct __attribute__((__packed__)) {
  XCOMHeader header;
  XCOMParHeader param_header;
  uint32_t mode;
  XCOMFooter footer;
} XCOMParGNSS_RTKMODE;

/**
 * This parameter sets the GNSS aiding frame.
 * The configured frame is valid for GNSS position and velocity aiding.
 */
#define GNSS_AIDFRAME_SELECT_ECEF 0
#define GNSS_AIDFRAME_SELECT_LLH 1
typedef struct __attribute__((__packed__)) {
  XCOMHeader header;
  XCOMParHeader param_header;
  uint32_t aiding_frame; /**< Aiding frame selection */
  XCOMFooter footer;
} XCOMParGNSS_AIDFRAME;

/**
 * This parameter enables or disables the RTCMv3 forwarding module.
 * Using this module, correction data can be sent to the system by wrapping it
 * in the PARGNSS_RTCMV3AIDING parameter. The INS forwards the received data
 * stream to the internal GNSS receiver after stripping the iXCOM frame wrapper.
 */
#define GNSS_RTCMV3CONFIG_DISABLE 0
#define GNSS_RTCMV3CONFIG_ENABLE 1
typedef struct __attribute__((__packed__)) {
  XCOMHeader header;
  XCOMParHeader param_header;
  uint8_t reserved1;
  uint8_t enable;
  uint16_t reserved2;
  uint32_t reserved3;
  XCOMFooter footer;
} XCOMParGNSS_RTCMV3CONFIG;

/**
 * This parameter can be used to exclude certain satellite constellations from
 * the GNSS solution computation.
 */
#define GNSS_LOCKOUTSYSTEM_MASK_GPS (1 << 0)
#define GNSS_LOCKOUTSYSTEM_MASK_GLONASS (1 << 1)
#define GNSS_LOCKOUTSYSTEM_MASK_SBAS (1 << 2)
#define GNSS_LOCKOUTSYSTEM_MASK_GALILEO (1 << 3)
#define GNSS_LOCKOUTSYSTEM_MASK_BEIDOU (1 << 4)
#define GNSS_LOCKOUTSYSTEM_MASK_QZSS (1 << 5)
#define GNSS_LOCKOUTSYSTEM_MASK_NAVIC (1 << 6)
typedef struct __attribute__((__packed__)) {
  XCOMHeader header;
  XCOMParHeader param_header;
  uint8_t lockoutsystem_mask; /**< Lockout System Mask */
  uint8_t reserved1;
  uint16_t reserved2;
  XCOMFooter footer;
} XCOMParGNSS_LOCKOUTSYSTEM;

/**
 * This parameter enables or disables the supply of electrical power
 * from the internal power source of the receiver
 */
#define GNSS_ANTENNAPOWER_SWITCH_OFF 0 /**< Disables antenna power */
#define GNSS_ANTENNAPOWER_SWITCH_ON 1  /**< Enables antenna power */
#define GNSS_ANTENNAPOWER_SWITCH_PRIMON_SECOFF                                 \
  3 /**< Enables primary antenna power and disables secondary antenna power */
#define GNSS_ANTENNAPOWER_SWITCH_PRIMOFF_SECON                                 \
  4 /**< Disables primary antenna power and enables secondary antenna power */
typedef struct __attribute__((__packed__)) {
  XCOMHeader header;
  XCOMParHeader param_header;
  uint32_t power_mask;
  XCOMFooter footer;
} XCOMParGNSS_ANTENNAPOWER;

/**
 * This parameter controls the PPS line of the internal
 * GNSS receiver
 */
#define GNSS_PPSCONTROL_SWITCH_DISABLE 0 /**< Disable PPS */
#define GNSS_PPSCONTROL_SWITCH_ENABLE 1  /**< Enable PPS */
#define GNSS_PPSCONTROL_SWITCH_ENABLE_FINETIME                                 \
  2 /**< Enable the PPS only when FINE or FINESTEERING time status has been    \
       reached (hw dependent) */
#define GNSS_PPSCONTROL_SWITCH_ENABLE_FINETIME_MINUTEALIGN                     \
  3 /**< Enable the PPS only when FINE or FINESTEERING time status has been    \
       reached AND the start of the next 60 seconds (1 minute modulus) has     \
       occurred (hw dependent) */

#define GNSS_PPSCONTROL_POLARITY_NEGATIVE 0
#define GNSS_PPSCONTROL_POLARITY_POSITIVE 1
typedef struct __attribute__((__packed__)) {
  XCOMHeader header;
  XCOMParHeader param_header;
  uint32_t pps_switch;
  uint32_t polarity; /**< Field to specify the polarity of the pulse to be
                        generated on the PPS output. */
  double period;     /**< Field to specify the period of the pulse, in [s] */
  uint32_t pulse_width_us; /**< Field to specify the pulse width of the PPS
                              signal in microseconds. */
  XCOMFooter footer;
} XCOMParGNSS_PPSCONTROL;

typedef struct __attribute__((__packed__)) {
  XCOMHeader header;
  XCOMParHeader param_header;
  float elevation_cutoff;
  uint8_t cn0_thresh_num_sv;
  uint8_t cn0_thresh;
  uint16_t reserved;
  XCOMFooter footer;
} XCOMParGNSS_NAVCONFIG;

typedef struct __attribute__((__packed__)) {
  XCOMHeader header;
  XCOMParHeader param_header;
  float stddev_scaling_pos;
  float stddev_min_pos;
  float stddev_scaling_rtk;
  float stddev_min_rtk;
  float stddev_scaling_vel;
  float stddev_min_vel;
  XCOMFooter footer;
} XCOMParGNSS_STDDEV;

typedef struct __attribute__((__packed__)) {
  XCOMHeader header;
  XCOMParHeader param_header;
  uint8_t enable;
  uint8_t debug_enable;
  uint16_t timeout;
  uint32_t voter_mode;
  float hysteresis;
  uint8_t portINT;
  uint8_t portEXT;
  uint16_t selection_mode;
  uint32_t baudINT;
  uint32_t baudEXT;
  XCOMFooter footer;
} XCOMParGNSS_VOTER;

typedef struct __attribute__((__packed__)) {
  uint8_t model_name[16];
  uint32_t year;
  uint32_t month;
  uint32_t day;
} XCOMParGNSS_MODELENTRIES;

typedef struct __attribute__((__packed__)) {
  XCOMHeader header;
  XCOMParHeader param_header;
  uint8_t rtk_code;
  uint8_t reserved1;
  uint16_t reserved2;
  XCOMParGNSS_MODELENTRIES models[6];
  XCOMFooter footer;
} XCOMParGNSS_MODEL;

typedef struct __attribute__((__packed__)) {
  XCOMHeader header;
  XCOMParHeader param_header;
  uint32_t component_type;
  uint8_t model[16];
  uint8_t psn[16];
  uint8_t hw_version[16];
  uint8_t sw_version[16];
  uint8_t boot_version[16];
  uint8_t comp_date[16];
  uint8_t comp_time[16];
  XCOMFooter footer;
} XCOMParGNSS_VERSION;

typedef struct __attribute__((__packed__)) {
  XCOMHeader header;
  XCOMParHeader param_header;
  struct {
    uint32_t set_mask;
    uint32_t clear_mask;
  } config[5];
  XCOMFooter footer;
} XCOMParGNSS_STATUSCONFIG;

typedef struct __attribute__((__packed__)) {
  XCOMHeader header;
  XCOMParHeader param_header;
  uint32_t position_type;
  XCOMFooter footer;
} XCOMParGNSS_RTKSOLTHR;

typedef struct __attribute__((__packed__)) {
  XCOMHeader header;
  XCOMParHeader param_header;
  uint8_t use_PPPPOS;
  uint8_t reserved[7];
  XCOMFooter footer;
} XCOMParGNSS_TERRASTAR;

/*
 * This parameter configures the GNSS reference station feature.
 * Field 1 of this parameter enables the Ntrip caster and server.
 * In order for changes to take effect, the configuration needs to be
 * saved and the device needs to be rebooted.
 */
#define GNSS_REFSTATION_NTRIP_DISABLE 0 /**< Disable Ntrip caster */
#define GNSS_REFSTATION_NTRIP_ENABLE 1  /**< Enable Ntrip caster */

#define GNSS_REFSTATION_FIXPOS_DISABLE                                         \
  0 /**< Position is not fixed in GNSS receiver */
#define GNSS_REFSTATION_FIXPOS_ENABLE 1 /**< Position is fixed in receiver */

#define GNSS_REFSTATION_RTCMV3OUT_DISABLE 0 /**< disable RTCM output */
#define GNSS_REFSTATION_RTCMV3OUT_ENABLE 1  /**< enable RTCM v 3.2 output */
typedef struct __attribute__((__packed__)) {
  XCOMHeader header;
  XCOMParHeader param_header;
  uint8_t enable_ntrip; /**< Enable/disable Ntrip caster/server */
  uint8_t use_fixpos;   /**< Use FIXPOS mode in internal GNSS receiver */
  uint8_t enable_rtcmv3_output; /**< Enable/disable RTCM output */
  uint8_t reserved1;
  XCOMFooter footer;
} XCOMParGNSS_REFSTATION;

/**
 * This parameter configures the fixed position of the GNSS receiver.
 * It can be set either via this parameter or by successful completion of
 * the position averaging procedure, which can be triggered via PARGNSS_POSAVE.
 */
typedef struct __attribute__((__packed__)) {
  XCOMHeader header;
  XCOMParHeader param_header;
  double fixpos[3]; /**< Fix position: longitude [rad], latitude [rad] and
                       altitude [m] */
  float fixpos_stdddev[3]; /**< Standard deviation of the entered fixed position
                              [m] */
  XCOMFooter footer;
} XCOMParGNSS_FIXPOS;

/*
 * This parameter configures the position averaging on the integrated GNSS
 * receiver.
 */
#define GNSS_POSAVE_AVGSTATUS_OFF 0 /**< Position averaging is turned off */
#define GNSS_POSAVE_AVGSTATUS_RUNNING                                          \
  1 /**< Position averaging is in progress and not yet finished */
#define GNSS_POSAVE_AVGSTATUS_COMPLETE                                         \
  2 /**< Position averaging process completed */
typedef struct __attribute__((__packed__)) {
  XCOMHeader header;
  XCOMParHeader param_header;
  float max_time; /**< Maximum amount of time that positions are to be averaged
                     [h] */
  float max_horizontal_stddev; /**< Desired horizontal standard deviation in [m]
                                */
  uint32_t averaging_status;   /**< Read-only status of the position averaging
                                  process */
  uint32_t averaging_time; /**< Elapsed time of averaging in [s] (read-only) */
  uint32_t
      averaging_samples; /**< Number of samples in the average (read-only ) */
  uint8_t reserved1[3];
  XCOMFooter footer;
} XCOMParGNSS_POSAVE;

/**
 * This parameter configures the correction data port of the
 * internal GNSS receiver
 * (DEPRECATED)
 */
typedef struct __attribute__((__packed__)) {
  XCOMHeader header;
  XCOMParHeader param_header;
  uint32_t rx_type;  /**< Receive interface mode */
  uint32_t tx_type;  /**< Transmit interface mode */
  uint32_t baudrate; /**< Baud rate of GNSS correction port */
  uint32_t enable;   /**< Enable correction port */
  float period_gga;  /**< Setup GGA period in [s]. */
  XCOMFooter footer;
} XCOMParGNSS_CORPORTCFG;

/*
 * This parameter configures the GNSS switcher module.
 */
#define PARGNSS_SWITCHER_INTERNAL_RECEIVER 0
#define PARGNSS_SWITCHER_EXTERNAL_RECEIVER 1
typedef struct __attribute__((__packed__)) {
  XCOMHeader header;
  XCOMParHeader param_header;
  uint8_t enable;
  uint8_t switcher;
  uint16_t reserved1;
  XCOMFooter footer;
} XCOMParGNSS_SWITCHER;

typedef struct __attribute__((__packed__)) {
  XCOMHeader header;
  XCOMParHeader param_header;
  float offset; /**< Offset in [rad] */
  XCOMFooter footer;
} XCOMParGNSS_HDGOFFSET;

/**
 * This parameter configures the serial port of the magnetometer module.
 */
typedef struct __attribute__((__packed__)) {
  XCOMHeader header;
  XCOMParHeader param_header;
  uint8_t port; /**< Serial port which is used for iMAG com- munication */
  uint8_t reserved2[3]; /**< Reserved for further use */
  uint32_t baudrate;    /**< Baud rate of the serial port */
  XCOMFooter footer;
} XCOMParMAG_PORT;

/**
 * This parameter configures the sampling rate of the magnetometer module
 */
typedef struct __attribute__((__packed__)) {
  XCOMHeader header;
  XCOMParHeader param_header;
  uint32_t period; /**< Sampling period of the magnetometer module in [ms] */
  XCOMFooter footer;
} XCOMParMAG_RATE;

/**
 * This parameter changes the misalignment of the magnetometer reference frame
 * with respect to the IMU reference frame. The angles contained in this
 * parameter are used to set up the IMU to magnetometer misalignment coordinate
 * transformation matrix.
 */
typedef struct __attribute__((__packed__)) {
  XCOMHeader header;
  XCOMParHeader param_header;
  float misalign[3]; /**< Rotation around x,y, and z-axis, respec- tively in
                        [rad] */
  XCOMFooter footer;
} XCOMParMAG_MISALIGN;

/**
 * This parameter contains the calibration parameters of the magnetometer.
 */
#define MAG_CALIBRATION_DISABLE 0
#define MAG_CALIBRATION_ENABLE 1
typedef struct __attribute__((__packed__)) {
  XCOMHeader header;
  XCOMParHeader param_header;
  float cc[9]; /**< Calibration matrix */
  float b[3];  /**< Bias vector */
  uint32_t calib_enable;
  XCOMFooter footer;
} XCOMParMAG_CAL;

/*
 * This parameter holds the on-board magnetometer calibration procedure status
 * of the 2D magnetometer calibration routine. A more accurate 3D calibration
 * routine is also available.
 */
#define MAG_CALSTATE_NONE 0     /* No calibration in progress */
#define MAG_CALSTATE_START 1    /* Start calibration, writable status */
#define MAG_CALSTATE_RUNNING 2  /* Calibration is currently running */
#define MAG_CALSTATE_RESERVED 3 /* Reserved for further use */
#define MAG_CALSTATE_BREAK 4    /* Stop calibration, writable status */
typedef struct __attribute__((__packed__)) {
  XCOMHeader header;
  XCOMParHeader param_header;
  int state; /* Magnetometer calibration status */
  XCOMFooter footer;
} XCOMParMAG_CALSTATE;

/*
 * This parameter contains the Figure Of Merit (FOM) of the 2D magnetometer
 * calibration method. It is read only.
 * The lower the resulting FOM value, the better is the qualitative
 * compensation.
 */
typedef struct __attribute__((__packed__)) {
  XCOMHeader header;
  XCOMParHeader param_header;
  float fom; /* Figure of merit (FOM) */
  XCOMFooter footer;
} XCOMParMAG_FOM;

/*
 * This parameter contains the magnetometer configuration items
 */
typedef struct __attribute__((__packed__)) {
  XCOMHeader header;
  XCOMParHeader param_header;
  struct {
    uint8_t enable : 1;     /* Enable/disable magnetometer module */
    uint8_t gimballing : 1; /* Enable/disable gimballing */
    uint8_t debug : 1;      /* Enable/disable debug output */
    uint8_t d03 : 1;        /* Reserved for further use */
    uint8_t d04 : 1;        /* Reserved for further use */
    uint8_t d05 : 1;        /* Reserved for further use */
    uint8_t d06 : 1;        /* Reserved for further use */
    uint8_t d07 : 1;        /* Reserved for further use */
  } cfg;
  uint8_t param_idx; /* EEPROM parameter index */
  uint16_t reserved1;
  XCOMFooter footer;
} XCOMParMAG_CFG;

/*
 * This parameter activates/deactivates the external magnetometer
 */
typedef struct __attribute__((__packed__)) {
  XCOMHeader header;
  XCOMParHeader param_header;
  uint32_t enable; /* Magnetometer activation */
  XCOMFooter footer;
} XCOMParMAG_ENABLE;

typedef struct __attribute__((__packed__)) {
  XCOMHeader header;
  XCOMParHeader param_header;
  uint32_t enable;
  XCOMFooter footer;
} XCOMParMADC_ENABLE;

typedef struct __attribute__((__packed__)) {
  XCOMHeader header;
  XCOMParHeader param_header;
  float lever_arm_xyz[3];
  float lever_arm_xyz_stddev[3];
  XCOMFooter footer;
} XCOMParMADC_LEVERARM;

typedef struct __attribute__((__packed__)) {
  XCOMHeader header;
  XCOMParHeader param_header;
  float cutoff_freq;
  uint32_t enable_filter;
  XCOMFooter footer;
} XCOMParMADC_LOWPASS;

/**
 * This parameter defines the odometer scale factor in [meter/tick].
 */
typedef struct __attribute__((__packed__)) {
  XCOMHeader header;
  XCOMParHeader param_header;
  float scf_odo;      /**< Odometer scaling factor in [meter/ticks] */
  float scf_odo_est;  /**< Estimated scaling factor in [meter/ticks] */
  uint32_t reserved1; /**< Reserved for further use */
  XCOMFooter footer;
} XCOMParODO_SCF;

/**
 * This parameter defines the odometer timeout in [sec].
 * If the odometer module does not detect any tick during the configured timeout
 * value PARODO_TIMEOUT, a ZUPT will be performed (only if odometer is enabled
 * via the activation mask of the PAREKF_ZUPT parameter. As timeout entry, only
 * values ≥ 0 seconds are accepted.
 */
typedef struct __attribute__((__packed__)) {
  XCOMHeader header;
  XCOMParHeader param_header;
  float timeout; /**< Odometer timeout in [sec], default: 5.0 */
  XCOMFooter footer;
} XCOMParODO_TIMEOUT;

#define ODO_MODE_DISABLE 0 /**< Disable odometer module */
#define ODO_MODE_ENBLE 1   /**< Enable odometer module */

#define ODO_MODE_A_COUNTER                                                     \
  0 /**< Rising edges on channel A are counted as ticks */
#define ODO_MODE_A_COUNTER_B_DIR                                               \
  1 /**< Rising edges on channel A are counted as ticks, the level of B        \
       channel at the the tick event indicates direction. */
#define ODO_MODE_AB_COUNTER_2QUAD                                              \
  2 /**< Rising and falling edges on channel A are counted as ticks, the level \
       of B channel at the tick event indicates direction. This doubles the    \
       resolution of the odometer compared to mode 1 */
#define ODO_MODE_AB_COUNTER_4QUAD                                              \
  3 /**< Rising and falling edges on both channels are counted as ticks, the   \
       level of the respective other channel at the tick event indicates the   \
       direction. This quadruples the resolution of the odometer compared to   \
       the first mode */
#define ODO_MODE_A_UP_B_DOWN                                                   \
  4 /**< Rising edges on channel A are counted up. Rising edges on channel B   \
       are counted down */
/**
 * This parameter defines the odometer mode and its related items.
 * For Deglitcher A and B, only values ≤ 255 internal ticks are accepted
 * (an internal tick has typically a value of 25ns).
 * In general, there is no need to change one of these values.
 */
typedef struct __attribute__((__packed__)) {
  XCOMHeader header;
  XCOMParHeader param_header;
  uint16_t enable;        /**< Odometer enable/disable switch */
  uint16_t mode;          /**< Odometer mode */
  uint16_t deglitcher[2]; /**< Deglitcher value of channel A/B in [internal
                             ticks], default: 10 */
  XCOMFooter footer;
} XCOMParODO_MODE;

typedef struct __attribute__((__packed__)) {
  XCOMHeader header;
  XCOMParHeader param_header;
  uint32_t swap_inputs;
  XCOMFooter footer;
} XCOMParODO_SWAP;

typedef struct __attribute__((__packed__)) {
  XCOMHeader header;
  XCOMParHeader param_header;
  uint16_t ppd_divider[3];
  uint16_t ppd_reserved;
  XCOMFooter footer;
} XCOMParODO_PPDDIVIDER;

typedef struct __attribute__((__packed__)) {
  XCOMHeader header;
  XCOMParHeader param_header;
  uint32_t mode;
  XCOMFooter footer;
} XCOMParODO_EQEP;

typedef struct __attribute__((__packed__)) {
  XCOMHeader header;
  XCOMParHeader param_header;
  float lever_arm[3];
  XCOMFooter footer;
} XCOMParODO_LEVERARM;

typedef struct __attribute__((__packed__)) {
  XCOMHeader header;
  XCOMParHeader param_header;
  float stddev;
  XCOMFooter footer;
} XCOMParODO_VELSTDDEV;

typedef struct __attribute__((__packed__)) {
  XCOMHeader header;
  XCOMParHeader param_header;
  float direction[3];
  XCOMFooter footer;
} XCOMParODO_DIRECTION;

typedef struct __attribute__((__packed__)) {
  XCOMHeader header;
  XCOMParHeader param_header;
  uint8_t enable;
  uint8_t reserved1;
  uint16_t reserved2;
  float constraints_stddev;
  XCOMFooter footer;
} XCOMParODO_CONSTRAINTS;

typedef struct __attribute__((__packed__)) {
  XCOMHeader header;
  XCOMParHeader param_header;
  float update_rate;
  XCOMFooter footer;
} XCOMParODO_RATE;

typedef struct __attribute__((__packed__)) {
  XCOMHeader header;
  XCOMParHeader param_header;
  float thr_acc;
  float thr_omg;
  XCOMFooter footer;
} XCOMParODO_THR;

typedef struct __attribute__((__packed__)) {
  XCOMHeader header;
  XCOMParHeader param_header;
  uint32_t mode;
  XCOMFooter footer;
} XCOMParODO_INVERTER;

typedef struct __attribute__((__packed__)) {
  XCOMHeader header;
  XCOMParHeader param_header;
  uint32_t port;
  XCOMFooter footer;
} XCOMParARINC825_PORT;

typedef struct __attribute__((__packed__)) {
  XCOMHeader header;
  XCOMParHeader param_header;
  uint32_t baud;
  XCOMFooter footer;
} XCOMParARINC825_BAUD;

typedef struct __attribute__((__packed__)) {
  XCOMHeader header;
  XCOMParHeader param_header;
  uint16_t reserved;
  uint16_t tx_enable;
  XCOMFooter footer;
} XCOMParARINC825_ENABLE;

typedef struct __attribute__((__packed__)) {
  XCOMHeader header;
  uint16_t param_id;
  uint8_t number_of_frames;
  uint8_t is_request;
  struct {
    uint16_t divider;
    uint16_t reserved;
    uint32_t doc; // see ARINC825 ICD
  } frame[XCOM_MAX_NUMBER_OF_ARINC825_FRAMES];
  XCOMFooter footer;
} XCOMParARINC825_LOGLIST;

typedef struct __attribute__((__packed__)) {
  XCOMHeader header;
  XCOMParHeader param_header;
  uint16_t bus_recovery;
  uint16_t reserved;
  XCOMFooter footer;
} XCOMParARINC825_BUSRECOVERY;

typedef struct __attribute__((__packed__)) {
  XCOMHeader header;
  XCOMParHeader param_header;
  uint16_t reset;
  uint16_t bus;
  XCOMFooter footer;
} XCOMParARINC825_RESETSTATUS;

typedef struct __attribute__((__packed__)) {
  XCOMHeader header;
  uint16_t param_id;
  uint8_t ucNumOfScaleFactors;
  uint8_t is_request;
  double dScfAcc;
  double dScfOmg;
  double dScfRPY[3];
  double dScfVel;
  double dScfTime;
  double dScfPos[3];
  double dScfRpyStdDev;
  double dScfInsPosStdDev;
  double dScfVelStdDev;
  double dScfGnssPosStdDev;
  XCOMFooter footer;
} XCOMParARINC825_SCALEFACTOR;

typedef struct __attribute__((__packed__)) {
  XCOMHeader header;
  XCOMParHeader param_header;
  uint32_t event_mask;
  XCOMFooter footer;
} XCOMParARINC825_EVENTMASK;

typedef struct __attribute__((__packed__)) {
  XCOMHeader header;
  XCOMParHeader param_header;
  uint8_t xcom_channel;
  uint8_t enable;
  uint16_t reserved;
  XCOMFooter footer;
} XCOMParREC_CONFIG;

typedef struct __attribute__((__packed__)) {
  XCOMHeader header;
  XCOMParHeader param_header;
  int8_t file_name[256];
  XCOMFooter footer;
} XCOMParREC_CURRENTFILE;

typedef struct __attribute__((__packed__)) {
  XCOMHeader header;
  XCOMParHeader param_header;
  uint8_t path[128];
  XCOMFooter footer;
} XCOMParREC_START;

typedef struct __attribute__((__packed__)) {
  XCOMHeader header;
  XCOMParHeader param_header;
  XCOMFooter footer;
} XCOMParREC_STOP;

typedef struct __attribute__((__packed__)) {
  XCOMHeader header;
  XCOMParHeader param_header;
  uint8_t suffix[128];
  XCOMFooter footer;
} XCOMParREC_SUFFIX;

typedef struct __attribute__((__packed__)) {
  XCOMHeader header;
  XCOMParHeader param_header;
  uint32_t enable;
  XCOMFooter footer;
} XCOMParREC_LOGROTATE;

typedef struct __attribute__((__packed__)) {
  XCOMHeader header;
  XCOMParHeader param_header;
  double free_space; /**< in MB */
  XCOMFooter footer;
} XCOMParREC_DISKSPACE;

#define EKF_VMP_MASK_POSITION 0x0001
#define EKF_VMP_MASK_VELOCITY 0x0002
#define EKF_VMP_MASK_SPECIFICFORCE 0x0004
/**
 * This parameter specifies a virtual measurement point.
 * The output position, velocity and acceleration may be transformed to
 * another point with fixed coordinates in the INS enclosure frame with
 * respect to the INS enclosure reference point (e.g. to a wheel or a camera
 * mounted on top of the vehicle).
 */
typedef struct __attribute__((__packed__)) {
  XCOMHeader header;
  uint16_t param_id;
  uint8_t channel_number; /**< Should always be set to '0' */
  uint8_t is_request;
  float leverarm[3]; /**< Distance between enclosure reference point and virtual
                        measurement point in x-, y-, and z-direction in [m] */
  uint16_t mask;     /**< Activation mask */
  uint16_t cut_off;  /**< This parameter specifies the cutoff frequency in [Hz]
                        of the first order lowpass, which is used to filter ω for
                        the transformation. Due to the noise, the derivative of ω
                        will not be used in the acceleration transformation. */
  XCOMFooter footer;
} XCOMParEKF_VMP;

#define EKF_AIDING_MODEMASK_GNSSPOS                                            \
  0x00000001 /**< GNSS Position Measurements */
#define EKF_AIDING_MODEMASK_BAROALT                                            \
  0x00000002 /**< Barometer Altitude Measurements */
#define EKF_AIDING_MODEMASK_HEIGHT 0x00000004 /**< Height Measurements */
#define EKF_AIDING_MODEMASK_GNSSVEL                                            \
  0x00000008 /**< GNSS Velocity Measurements */
#define EKF_AIDING_MODEMASK_BODYVEL                                            \
  0x00000010 /**< 3D Body Velocity Measurements */
#define EKF_AIDING_MODEMASK_ODOMETER 0x00000020 /**< Odometer Measurements */
#define EKF_AIDING_MODEMASK_TAS 0x00000040 /**< True Air Speed Measurements */
#define EKF_AIDING_MODEMASK_DUALANTHDG                                         \
  0x00000080 /**< Heading Measurements (from dual antenna GNSS) */
#define EKF_AIDING_MODEMASK_MAGHDG                                             \
  0x00000100 /**< Magnetic Heading Measurements (from Magnetic Field           \
                Measurements) */
#define EKF_AIDING_MODEMASK_MAGFIELD                                           \
  0x00000200 /**< Magnetic Field Measurements */
#define EKF_AIDING_MODEMASK_BSLXYZ                                             \
  0x00000400 /**< Baseline XYZ Measurements                                    \
              */
#define EKF_AIDING_MODEMASK_GNSSPSR                                            \
  0x00000800 /**< GNSS Pseudorange Measurements */
#define EKF_AIDING_MODEMASK_GNSSRR                                             \
  0x00001000                                /**< GNSS Range Rate Measurements */
#define EKF_AIDING_MODEMASK_TCPD 0x00002000 /**< TCPD Measurements */
#define EKF_AIDING_MODEMASK_TCPDDD 0x00004000 /**< TCPD_DD Measurements */
#define EKF_AIDING_MODEMASK_ODOALONGTRK                                        \
  0x00008000 /**< Odometer Along-Track Measurement */
#define EKF_AIDING_MODEMASK_EXTPOS                                             \
  0x00010000 /**< External Position Measurements */
#define EKF_AIDING_MODEMASK_EXTVEL                                             \
  0x00020000 /**< External Velocity Measurements */
#define EKF_AIDING_MODEMASK_GRAVITY 0x00040000 /**< Gravity Aiding */
/**
 * This parameter defines the used aiding sources of the extended Kalman filter.
 * The measurements of those bits, which are set in the mode field, are used as
 * aiding source. All other data sources are ignored in the data fusion. Per
 * default, all aiding sources are activated, resulting in a aiding mode mask of
 * 0xFF FF FF FF.
 */
typedef struct __attribute__((__packed__)) {
  XCOMHeader header;
  XCOMParHeader param_header;
  uint32_t mode; /**< Bitfield with aiding sources to use */
  uint32_t
      mask; /**< An activation mask for changing the Aid- ingMode bitfield. Only
               these values are changed for which the mask is enabled. */
  XCOMFooter footer;
} XCOMParEKF_AIDING;

/**
 * This parameter defines the delay in [ms] of the delayed navigation module
 * against the real-time navigator. Since the aiding data received by the IMS is
 * generally only available delayed, the delay must be handled by a special
 * buffering mechanism. With this param- eter, the buffer depth and thus maximum
 * delay can be adjusted. The buffer depth should be kept as low as necessary.
 * Due to the buffer size of the realtime navigator, the delay value is limited
 * to < 490 ms.
 */
typedef struct __attribute__((__packed__)) {
  XCOMHeader header;
  XCOMParHeader param_header;
  uint32_t delay; /**< Delay in [ms] */
  XCOMFooter footer;
} XCOMParEKF_DELAY;

/**
 * This parameter defines the thresholds of the Alignment Status and Position
 * Accuracy fields of the global status The configurable thresholds specify the
 * flagged position and heading accuracies. The thresholds are internally
 * compared against the standard deviations obtained from the extended Kalman
 * filter. If the heading standard deviation falls below ThrHDG (field 5) during
 * static alignment, alignment will automatically terminated and the INS is
 * ready to move.
 */
typedef struct __attribute__((__packed__)) {
  XCOMHeader header;
  XCOMParHeader param_header;
  float thr_heading; /**< Threshold for heading fields of alignment status in
                        [rad] */
  float thr_pos_med; /**< Threshold for POS_MEDIUM_ACCURACY field of position
                        accuracy in [m] */
  float thr_pos_hi;  /**< Threshold for POS_HIGH_ACCURACY field of position
                        accuracy in [m] */
  XCOMFooter footer;
} XCOMParEKF_HDGPOSTHR;

/**
 * A smoothed position is needed by many control systems.
 * The IMS offers the possibility to smooth the output of the real-time
 * navigation task. The PAREKF_SMOOTH parameter defines the smoothing factor in
 * [samples]. If a position/velocity/attitude jump is detected by the IMS, the
 * jump will be distributed over SMOOTH samples
 */
typedef struct __attribute__((__packed__)) {
  XCOMHeader header;
  XCOMParHeader param_header;
  uint32_t slew_samples; /**< Smoothing factor in [samples] */
  XCOMFooter footer;
} XCOMParEKF_SMOOTH;

#define EKF_ZUPT_MASK_ACCEL 0 /**< Acceleration */
#define EKF_ZUPT_MASK_GYRO 1  /**< Angular rate */
#define EKF_ZUPT_MASK_ODO 2   /**< Odometer measurements */
/**
 * The PAREKF_ZUPT parameter contains the configuration of the zero velocity
 * detector. Possible sources for the zero velocity detection are wheel speed
 * measurements, angular rate and acceleration measurements. The respective
 * sources are activated by the Activation mask field (field 14) of this
 * parameter. The angular rate and acceleration measurement are low-pass
 * filtered prior to comparing them to the detection threshold. The cut-off
 * frequency for this low-pass filter may bet set by the CutOffFreq field (field
 * 8) of this parameter. A zero velocity condition is detected if the values of
 * all selected sources have been below their respective threshold for longer
 * than the delay field (field 13) of this parameter. The status of zero
 * velocity detection will be reported in the In-Motion bit of the EXTSYSSTAT
 * field of the SYS_STAT message. A detected zero velocity condition will lead
 * to aiding the EKF with zero velocity updates if the AutoZUPT field (field 15)
 * of this parameter is set. The rate for this zero velocity aiding is
 * determined by the ZUPTRate field (field 9) of this parameter.
 */
typedef struct __attribute__((__packed__)) {
  XCOMHeader header;
  XCOMParHeader param_header;
  double thr_acc; /**< Acceleration threshold in [m/s2 ] */
  double thr_omg; /**< Angular Rate threshold in [rad/sec] */
  double thr_vel; /**< Odometer velocity threshold in [m/s] */
  float cutoff; /**< Accelerations and angular rates are low- pass filtered and
                   the consequential data will be compared to the configured
                   thresholds. The parameter unit is [Hz] */
  float zupt_period;     /**< ZUPT interval in [sec] */
  float min_stddev_zupt; /**< Final ZUPT StdDev in [m/s]. Starting with a higher
                            standard deviation, it will decrease until hitting
                            this limitation. */
  float weighting_factor; /**< Weighting factor of ZUPT StdDev filter */
  float time_constant;    /**< Time constant in [sec] */
  uint16_t delay;         /**< Delay in [ms] */
  uint8_t mask;           /**< Activation mask */
  uint8_t automatic_zupt; /**< Automatic ZUPT aiding */
  XCOMFooter footer;
} XCOMParEKF_ZUPT;

#define EKF_OUTLIER_MODEMASK_GNSSPOS                                           \
  0x00000001 /**< GNSS Position Measurements */
#define EKF_OUTLIER_MODEMASK_BAROALT                                           \
  0x00000002 /**< Barometer Altitude Measurements */
#define EKF_OUTLIER_MODEMASK_HEIGHT 0x00000004 /**< Height Measurements */
#define EKF_OUTLIER_MODEMASK_GNSSVEL                                           \
  0x00000008 /**< GNSS Velocity Measurements */
#define EKF_OUTLIER_MODEMASK_BODYVEL                                           \
  0x00000010 /**< 3D Body Velocity Measurements */
#define EKF_OUTLIER_MODEMASK_ODOMETER 0x00000020 /**< Odometer Measurements */
#define EKF_OUTLIER_MODEMASK_TAS                                               \
  0x00000040 /**< True Air Speed Measurements                                  \
              */
#define EKF_OUTLIER_MODEMASK_DUALANTHDG                                        \
  0x00000080 /**< Heading Measurements (from dual antenna GNSS) */
#define EKF_OUTLIER_MODEMASK_MAGHDG                                            \
  0x00000100 /**< Magnetic Heading Measurements (from Magnetic Field           \
                Measurements) */
#define EKF_OUTLIER_MODEMASK_MAGFIELD                                          \
  0x00000200 /**< Magnetic Field Measurements */
#define EKF_OUTLIER_MODEMASK_BSLXYZ                                            \
  0x00000400 /**< Baseline XYZ Measurements                                    \
              */
#define EKF_OUTLIER_MODEMASK_GNSSPSR                                           \
  0x00000800 /**< GNSS Pseudorange Measurements */
#define EKF_OUTLIER_MODEMASK_GNSSRR                                            \
  0x00001000 /**< GNSS Range Rate Measurements */
#define EKF_OUTLIER_MODEMASK_TCPD 0x00002000   /**< TCPD Measurements */
#define EKF_OUTLIER_MODEMASK_TCPDDD 0x00004000 /**< TCPD_DD Measurements */
#define EKF_OUTLIER_MODEMASK_ODOALONGTRK                                       \
  0x00008000 /**< Odometer Along-Track Measurement */
#define EKF_OUTLIER_MODEMASK_EXTPOS                                            \
  0x00010000 /**< External Position Measurements */
#define EKF_OUTLIER_MODEMASK_EXTVEL                                            \
  0x00020000 /**< External Velocity Measurements */
#define EKF_OUTLIER_MODEMASK_GRAVITY 0x00040000 /**< Gravity Aiding */
/**
 * This parameter defines the outlier rejection mask of the integrated Kalman
 * filter. The outliers of the measurements selected in the mode field, are
 * detected and excluded from EKF aiding. Per default, all outlier detections
 * are activated, resulting in a outlier mode mask of 0xFF FF FF FF.
 */
typedef struct __attribute__((__packed__)) {
  XCOMHeader header;
  XCOMParHeader param_header;
  uint32_t mode; /**< Outlier selection */
  uint32_t mask; /**< An activition mask for changing the 'mode' bitfield. Only
                    these values are changed for which the mask is enabled. */
  XCOMFooter footer;
} XCOMParEKF_OUTLIER;

#define EKF_STARTUPV2_POSMODE_GNSS                                                 \
  0 /**< The initial position will be set via the GNSS solution. If no valid       \
       GNSS data are available, the position will be loaded from the internal      \
       memory, when the GNSS timeout has expired. If the INS cannot load the       \
       stored position, the indicator “EKF SavedPos Error” will be set and the \
       default position at PAREKF_DEFPOS will be loaded (see behaviour at          \
       “STOREDPOS”). */
#define EKF_STARTUPV2_POSMODE_STORED                                              \
  1 /**< The initial position will be loaded from the internal storage. If the    \
       IMS cannot load the saved position, the indicator “EKF SavedPos Error” \
       will be set. The default position PAREKF_DEFPOS will be loaded as the      \
       best of all worse solutions. */
#define EKF_STARTUPV2_POSMODE_FORCED                                           \
  2 /**< The transmitted position will be used to set the initial position of  \
       the Kalman filter. If the transmitted position is out of range, the     \
       stored position will be loaded from the internal memory. If the IMS     \
       cannot load the stored position, the indicator “EKF SavedPos Error” \
       will be set and the default position PAREKF_DEFPOS will be used (see    \
       behaviour at “STOREDPOS”). */
#define EKF_STARTUPV2_POSMODE_CURRENT                                          \
  3 /**< The current position (last result of the navigation filter) will be   \
       used as initial position. This option should only be selected for a     \
       realignment. */

#define EKF_STARTUPV2_HDGMODE_UNKNOWN 0 /**< The initial heading is unknown */
#define EKF_STARTUPV2_HDGMODE_STORED                                             \
  1 /**< The initial heading will be loaded from the internal storage. If the    \
       IMS cannot load the saved heading, the indicator “EKF SavedHdg Error” \
       will be set. In this case, the IMS will be set to the default heading     \
       “DEFAULTHDG”. */
#define EKF_STARTUPV2_HDGMODE_FORCED                                           \
  2 /**< The initial heading will be transmitted via the PAREKF_STARTUP        \
       parameter. If the IMS does not receive a valid heading, the system will \
       answer with an error message and the heading will be set to             \
       “STOREDHDG”. */
#define EKF_STARTUPV2_HDGMODE_MAG                                              \
  3 /**< The initial heading will be set by the magnetometer value (magnetic   \
       heading). If the magnetic is not available, the heading will be set to  \
       “STOREDHDG”. */
#define EKF_STARTUPV2_HDGMODE_DUALANT                                          \
  4 /**< The initial heading will be set by the dual antenna GNSS receiver     \
       with the first valid GNSS sample. Until the GNSS data are valid, the    \
       heading will remain uninitialized; after the GNSS timeout has expired   \
       without a valid solution, the “STOREDHDG” will be used. */

#define EKF_STARTUPV2_ALTMSL_DISABLE 0 /**< Disabled */
#define EKF_STARTUPV2_ALTMSL_ENABLE                                            \
  1 /**< Enabled, regarding geoid undulation */

#define EKF_STARTUPV2_RESTART_DISABLE 0 /**< Disable automatic restart */
#define EKF_STARTUPV2_RESTART_ENABLE 1  /**< Enable automatic restart */
/**
 * This parameter can be used to define the initialization behavior of the
 * Kalman filter. A state chart detailing the navigation startup process is
 * available in appendix of DOC141126064
 */
typedef struct __attribute__((__packed__)) {
  XCOMHeader header;
  XCOMParHeader param_header;
  double initpos_lon;      /**< Initial longitude position in WGS84 in [rad] */
  double initpos_lat;      /**< Initial latitude position in WGS84 in [rad] */
  float initpos_alt;       /**< Initial altitude in WGS84 in [m] */
  float initpos_stddev[3]; /**< Standard deviation of initial position and
                              altitude in [m] */
  float inithdg;           /**< Initial heading in [rad] (in NED) Range: ±π */
  float inithdg_stdddev; /**< Standard deviation of initial heading in [rad] */
  float lever_arm[3];    /**< Lever arm in x-, y- and z-direction in [m] */
  float lever_arm_stddev[3]; /**< Lever arm standard deviation in x-, y- and
                                z-direction in [m] */
  uint8_t position_mode;     /**< Position initialization mode */
  uint8_t hdg_mode;          /**< Heading initialization mode */
  uint16_t
      gnss_timeout; /**< GNSS timeout in [sec] Set to ’0’ to wait forever */
  uint8_t enable_alt_msl;    /**< Alternate Mean Sea */
  uint8_t realignment;       /**< 0: Store parameters without execution of an
                                alignment; 1: Start alignment immediately with the
                                transmitted alignment parameters */
  uint8_t forced_inmotion;   /**< During alignment, system is 0: Static; 1:
                                In-motion */
  uint8_t automatic_restart; /**< The levelling will be restarted if movement of
                                the system is detected during the levelling
                                phase. The fine alignment will be finished if
                                movement of the system is detected */
  XCOMFooter footer;
} XCOMParEKF_STARTUPV2;

/**
 * This parameter defines the default position.
 * The circumstances under which the default position will be loaded are
 * detailed in the state charts in DOC141126064. The parameter entries only
 * accept values of a certain range, for a detailed description see the
 * PAREKF_STARTUPV2 parameter
 */
typedef struct __attribute__((__packed__)) {
  XCOMHeader header;
  XCOMParHeader param_header;
  double lon; /**< Default longitude position in WGS84 in [rad] */
  double lat; /**< Default latitude position in WGS84 in [rad] */
  float alt;  /**< Default altitude in [m] */
  XCOMFooter footer;
} XCOMParEKF_DEFPOS;

/**
 * This parameter defines the default heading. The default heading will be
 * loaded, if the stored heading is invalid. The parameter entries only accept
 * values of a certain range, for a detailed description see the
 * PAREKF_STARTUPV2 parameter
 */
typedef struct __attribute__((__packed__)) {
  XCOMHeader header;
  XCOMParHeader param_header;
  float default_hdg; /**< Default heading in NED frame in [rad] */
  XCOMFooter footer;
} XCOMParEKF_DEFHDG;

#define EKF_POWERDOWN_STATE_NOTSTORE                                           \
  0 /**< Do not save position and attitude                                     \
     */
#define EKF_POWERDOWN_STATE_STORE                                              \
  1 /**< Save position and attitude to the internal storage */
/*
 * This parameter defines the power-down behavior of the system.
 * If the 'store_state' field is set to "1", the systems will store its current
 * position and attitude state to the internal storage.
 * The stored position/attitude can be loaded during the next alignment.
 */
typedef struct __attribute__((__packed__)) {
  XCOMHeader header;
  XCOMParHeader param_header;
  uint32_t store_state; /**< power-down behavior of the system */
  XCOMFooter footer;
} XCOMParEKF_POWERDOWN;

/**
 * This parameter is read-only and contains the earth radii.
 */
typedef struct __attribute__((__packed__)) {
  XCOMHeader header;
  XCOMParHeader param_header;
  float radii[2]; /**< Earth radii for M and N configuration in [m]. */
  XCOMFooter footer;
} XCOMParEKF_EARTHRAD;

/**
 * This read-only parameter provides the stored position and the associated
 * standard devi- ation.
 */
typedef struct __attribute__((__packed__)) {
  XCOMHeader header;
  XCOMParHeader param_header;
  double lon;        /**< Longitude in [rad] */
  double lat;        /**< Latitude in [rad] */
  double alt;        /**< Altitude in [m] */
  double lon_stddev; /**< Longitude standard deviation in [m] */
  double lat_stddev; /**< Latitude standard deviation in [m] */
  double alt_stddev; /**< Altitude standard deviation in [m] */
  XCOMFooter footer;
} XCOMParEKF_STOREDPOS;

/**
 * This parameter is read-only and contains the stored attitude.
 */
typedef struct __attribute__((__packed__)) {
  XCOMHeader header;
  XCOMParHeader param_header;
  float rpy[3];        /**< Stored angels for roll, pitch and yaw in [rad] */
  float rpy_stddev[3]; /**< Standard deviation of stored angels roll, pitch and
                          yaw in [rad] */
  XCOMFooter footer;
} XCOMParEKF_STOREDATT;

/**
 * This parameter configures the standard deviation threshold of the position
 * aiding module. GNSS solutions with a standard deviation greater than this
 * threshold will be discarded from aiding. The standard deviation value only
 * accepts positive values > 0.
 */
typedef struct __attribute__((__packed__)) {
  XCOMHeader header;
  XCOMParHeader param_header;
  double thr_stddev_pos; /**< Standard deviation threshold of the position
                            aiding module. If the standard deviation is larger
                            than the configured threshold, the measurement
                            update will be refused. */
  XCOMFooter footer;
} XCOMParEKF_POSAIDSTDDEVTHR;

#define EKF_SCHULERMODE_DISABLE 0 /**< Disable schuler mode */
#define EKF_SCHULERMODE_ENABLE 1  /**< Enable schuler mode */
/**
 * This parameter configures the system for Schuler Mode test.
 */
typedef struct __attribute__((__packed__)) {
  XCOMHeader header;
  XCOMParHeader param_header;
  uint32_t schuler_mode; /**< Schuler Mode test configuration */
  XCOMFooter footer;
} XCOMParEKF_SCHULERMODE;

#define EKF_ODOMETER_INNOAVG_DISABLE 0 /**< Disable innovation averaging */
#define EKF_ODOMETER_INNOAVG_ENABLE                                            \
  1 /**< Enable innovation averaging (default) */

#define EKF_ODOMETER_COARSCALIB_DISABLE                                        \
  0 /**< Disable coarse calibration mode */
#define EKF_ODOMETER_COARSCALIB_ENABLE                                         \
  1 /**< Enable coarse calibration mode                                        \
     */
/**
 * This parameter provides advanced odometer configuration options which can be
 * used to accommodate for odometer signal abnormalities. Please contact iMAR
 * support (support@imar- navigation.de) to assist you with the right settings
 * should there be any problems with your signal.
 */
typedef struct __attribute__((__packed__)) {
  XCOMHeader header;
  XCOMParHeader param_header;
  float scale_factor_error; /**< Scale factor error in [%] */
  float
      scale_factor_stddev; /**< Scale factor error standard deviation in [%] */
  float scale_factor_rw;   /**< Random walk scale factor in [1/s/sqrt(Hz)] */
  float misalignment[2];   /**< Start values of misalignment in [rad] */
  float misalignment_stddev[2]; /**< Standard deviation of start values of mis-
                                   alignment in [rad] */
  float misalignment_rw[2]; /**< Random walk misalignment in [rad/s/sqrt(Hz)] */
  float min_velocity;       /**< Minimum velocity in [m/s], default: 0.1 */
  float max_velocity;       /**< Maximum velocity in [m/s], default: 100 */
  uint16_t inno_avg_enable; /**< Innovation averaging */
  uint16_t coarse_calib_enable; /**< Coarse calibration mode */
  XCOMFooter footer;
} XCOMParEKF_ODOMETER;

#define EKF_ODOBOGIE_COMP_DISABLE 0 /**< Disable bogie compensation */
#define EKF_ODOBOGIE_COMP_ENABLE 1  /**< Enable bogie compensation */
/**
 * This parameter configures the odometer data processing module for railway
 * applications. By enabling this parameter, the carriage box to bogie
 * misalignment angles which occur in curves, will be compensated. The distance
 * between the two bogies of the carriage box where the INS is installed has to
 * be configured. As Bogie Distance, only non-negative values ≥ 0 are accepted.
 */
typedef struct __attribute__((__packed__)) {
  XCOMHeader header;
  XCOMParHeader param_header;
  float bogie_distance;               /**< Bogie distance in [m] */
  uint32_t enable_bogie_compensation; /**< Enable/Disable bogie compensation */
  XCOMFooter footer;
} XCOMParEKF_ODOBOGIE;

#define EKF_GNSSLEVERARMEST_PRIM_DISABLE                                       \
  0 /**< Disable lever arm estimation for primary antenna */
#define EKF_GNSSLEVERARMEST_PRIM_ENABLE                                        \
  1 /**< Enable lever arm estimation for primary antenna */
#define EKF_GNSSLEVERARMEST_SEC_DISABLE                                        \
  0 /**< Disable lever arm estimation for secondary antenna */
#define EKF_GNSSLEVERARMEST_SEC_ENABLE                                         \
  1 /**< Enable lever arm estimation for secondary antenna */
/**
 * This parameter configures GNSS lever arm estimation.
 * GNSS lever arm estimation is only possible if the GNSS receiver supports RTK
 * operation and is able to fix the RTK solution. RTK capability can be checked
 * with the PARGNSS_MODEL parameter.
 */
typedef struct __attribute__((__packed__)) {
  XCOMHeader header;
  XCOMParHeader param_header;
  uint16_t primary_antenna;   /**< Primary antenna configuration */
  uint16_t secondary_antenna; /**< Secondary antenna configuration */
  XCOMFooter footer;
} XCOMParEKF_GNSSLEVERARMEST;

/**
 * This parameter configures the GNSS aiding rates inside the Kalman filter.
 */
typedef struct __attribute__((__packed__)) {
  XCOMHeader header;
  XCOMParHeader param_header;
  uint16_t psrpos_period; /**< Aiding interval pseudorange position in [sec] */
  uint16_t psrvel_period; /**< Aiding interval pseudorange velocity in [sec] */
  uint16_t rtk_period;    /**< Aiding interval RTK in [sec] */
  uint16_t rtk_timeout;   /**< Aiding interval RTK timeout in [sec] */
  uint16_t hdg_period;    /**< Dual antenna heading update interval in [sec] */
  uint16_t zupt_timeout;  /**< Zero Velocity Update timeout in [sec] */
  XCOMFooter footer;
} XCOMParEKF_GNSSAIDRATE;

/**
 * This parameter configures the PDOP threshold inside the Kalman filter.
 * GNSS solutions with a PDOP worse than this threshold will be discarded from
 * aiding. As PDOP threshold only positive values > 0 are accepted.
 */
typedef struct __attribute__((__packed__)) {
  XCOMHeader header;
  XCOMParHeader param_header;
  float pdop; /**< PDOP Threshold */
  XCOMFooter footer;
} XCOMParEKF_GNSSPDOP;

#define EKF_DUALANTAID_MODE_INIT                                               \
  0 /**< Initialization mode: The GNSS heading will be used to set the initial \
       heading. */
#define EKF_DUALANTAID_MODE_INTERVAL                                           \
  1 /**< Interval mode: The GNSS heading aiding will be performed with a fixed \
       rate. */
#define EKF_DUALANTAID_MODE_AUTO                                               \
  2 /**< Automatic mode (recommended): The GNSS heading aiding will be         \
       performed with a fixed rate until the INS yaw standard deviation        \
       reaches the configured threshold. */
/**
 * This parameter configures the dual antenna aiding inside the Kalman filter.
 */
typedef struct __attribute__((__packed__)) {
  XCOMHeader header;
  XCOMParHeader param_header;
  float threshold_heading_stddev; /**< GNSS heading standard deviation threshold
                                     in [rad]. If the received GNSS heading
                                     standard deviation is larger than the
                                     configured threshold, the measurement
                                     update will not be performed. */
  float threshold_pitch_stddev; /**< GNSS pitch standard deviation threshold in
                                   [rad]. If the received GNSS pitch standard
                                   deviation is larger than the configured
                                   threshold, the measurement update will not be
                                   performed. */
  float threshold_ins_yaw_stddev; /**< INS yaw standard deviation threshold in
                                     [rad]. If aiding mode 2 is configured, GNSS
                                     heading updates will be performed until the
                                     yaw standard deviation reaches the
                                     configured threshold. */
  uint32_t aiding_mode;           /**< The configured aiding mode */
  XCOMFooter footer;
} XCOMParEKF_DUALANTAID;

#define EKF_MAGATTAID_MODE_INIT                                                \
  0 /**< Initialization mode: The iMAG heading will be used to set the initial \
       heading */
#define EKF_MAGATTAID_MODE_INTEVAL                                             \
  1 /**< Interval mode: The iMAG heading aiding will be performed with a fixed \
       rate. */
#define EKF_MAGATTAID_MODE_AUTO                                                \
  2 /**< Automatic mode (recommended): The iMAG heading aiding will be         \
       performed with a fixed rate until the INS yaw standard. deviation       \
       reaches the configured threshold */
#define EKF_MAGATTAID_MODE_NAVAID                                              \
  3 /**< Navigation mode: iMAG heading aiding will be performed once the EKF   \
       was able to determine initial heading to an accuracy better than 2      \
       degrees (0.03491 rad). This prevents faulty initialization in case of   \
       possible unknown magnetic disturbances. */
/**
 * This parameter configures the iMAG parameters inside the Kalman filter.
 */
typedef struct __attribute__((__packed__)) {
  XCOMHeader header;
  XCOMParHeader param_header;
  uint32_t sample_period; /**< Sampling period of the magnetometer measurements
                             in [ms] */
  float hdg_stddev;  /**< Standard deviation of the magnetometer heading aiding
                        in [rad] */
  float latency;     /**< Latency of magnetometer measurement in [s] */
  float ins_yaw_thr; /**< Threshold of the integrated heading solution (used for
                        automatic aiding mode) in [rad] */
  uint8_t aiding_mode;       /**< Configures the aiding behavior. */
  uint8_t update_mode;       /**< EKF update mode */
  uint16_t aiding_interval;  /**< Magnetometer aiding interval in [s] */
  float mag_field_stddev[3]; /**< Magnetic field standard deviation in [mG] */
  XCOMFooter footer;
} XCOMParEKF_MAGATTAID;

/**
 * This parameter configures the iMADC parameters inside the Kalman filter.
 */
typedef struct __attribute__((__packed__)) {
  XCOMHeader header;
  XCOMParHeader param_header;
  float altitude_stddev;     /**< Altitude standard deviation in [m] */
  float latency;             /**< Latency in [sec] */
  uint16_t aiding_interval;  /**< Aiding interval in [sec] */
  uint16_t reserved;         /**< Reserved for further use */
  float scale_factor_error;  /**< Baro scale factor error */
  float scale_factor_stddev; /**< Baro scale factor error standard devation */
  float scale_factor_rw; /**< Random walk of scale factor in [1/s/sqrt(Hz)] */
  float bias;            /**< Barometric offset in [m] */
  float bias_stddev;     /**< Barometric offset standard deviation in [m] */
  float bias_rw; /**< Random walk of barometric offset in [m/s/sqrt(Hz)] */
  XCOMFooter footer;
} XCOMParEKF_MADCAID;

#define EKF_ALIGNMENT_METHOD_STATIONARY 0 /**< Stationary alignment mode */
#define EKF_ALIGNMENT_METHOD_DYNAMIC 1    /**< Dynamic alignment mode */

#define EKF_ALIGNMENT_GYROAVG_DISABLE 0 /**< Disable gyro averaging */
#define EKF_ALIGNMENT_GYROAVG_ENABLE 1  /**< Enable gyro averaging */

#define EKF_ALIGNMENT_TRACKALIGN_DISABLE 0 /**< Disable track alignment */
#define EKF_ALIGNMENT_TRACKALIGN_ENABLE 1  /**< Enable track alignment */
/**
 * This parameter configures the alignment method of the INS. The default mode
 * is different for every device and should only be changed by advanced users.
 *
 * The INS supports two general modes of alignment:
 * Stationary alignment: Requires the INS to be in a stationary condition during
 * alignment (small disturbances e.g. due to wind gusts are allowed) The
 * alignment starts with a levelling phase with a duration controlled by the
 * LevellingDuration field of this parameter. Afterwards, zero velocity updates
 * are performed to refine the estimates of attitude (and heading for device
 * with gyro compassing capability). Af- ter a duration controlled by the
 * StationaryDuration field of this parameter, these automated zero velocity
 * updates are terminated and the INS may be moved. The standard deviation in
 * field 7 of this parameter may be adjusted for e.g. high vibration
 * environments.
 *
 * In-Motion alignment: Does not impose any requirements for the motion
 * experienced by the INS during alignment, i.e. the INS may be in motion during
 * alignment. GNSS position and velocity have to be available during alignment.
 * The Alignment routine automatically completes once heading and attitude have
 * been determined with sufficient accuracy (depending on the device and
 * trajectory dynamics, the attitude standard deviations for all axes have to be
 * between 0 and 5 degrees (0 and 0.08727 rad)). After transition to navigation
 * mode, heading and attitude will be refined further.
 */
typedef struct __attribute__((__packed__)) {
  XCOMHeader header;
  XCOMParHeader param_header;
  uint32_t align_method;             /**< Alignment mode */
  uint16_t levelling_duration;       /**< Levelling duration in [sec] */
  uint16_t stationaryalign_duration; /**< Stationary duration in [sec] */
  double align_zupt_stddev;   /**< Alignment ZUPT standard deviation in [m/s] */
  uint8_t enable_gyro_avg;    /**< Gyro averaging */
  uint8_t enable_track_align; /**< Track alignment */
  float track_align_thr;      /**< Track alignment threshold in [m/s] */
  float track_align_direction[3]; /**< Track alignment direction for x-, y- and
                                     z-axis in [m/s] */
  uint16_t reserved_1;            /**< Reserved for further use */
  uint32_t reserved_2[3];         /**< Reserved for further use */
  XCOMFooter footer;
} XCOMParEKF_ALIGNMENT;

#define EKF_GRAVITYAIDING_DISABLE 0 /**< Disable gravity aiding */
#define EKF_GRAVITYAIDING_ENABLE 1  /**< Enable gravity aiding */
/**
 * This parameter configures the gravity aiding inside the Kalman filter.
 */
typedef struct __attribute__((__packed__)) {
  XCOMHeader header;
  XCOMParHeader param_header;
  uint32_t enable;       /**< Enable/Disable gravity aiding */
  float omg_thr;         /**< Angular rate sensor threshold in [rad/s] */
  float acc_thr;         /**< Accelerometer threshold in [m/s2] */
  float stddev;          /**< Standard deviation in [rad] */
  float gnss_timeout;    /**< GNSS timeout in [sec] */
  float aiding_interval; /**< Aiding interval in [sec] */
  XCOMFooter footer;
} XCOMParEKF_GRAVITYAIDING;

#define EKF_FEEDBACK_MASK_POSITION 0x00000001
#define EKF_FEEDBACK_MASK_VELOCITY 0x00000002
#define EKF_FEEDBACK_MASK_ATTITUDE 0x00000004
#define EKF_FEEDBACK_MASK_SENSORERORS 0x00000008
/**
 * This parameter enables or disables the error compensation in the real-time
 * navigation module. Setting a certain bit in this mask to ’0’ disables the
 * feedback. Per default, all feedbacks to realtime navigation are activated,
 * resulting in a feedback mode mask of ’0xFFFF’.
 */
typedef struct __attribute__((__packed__)) {
  XCOMHeader header;
  XCOMParHeader param_header;
  uint32_t feedback; /**< Feedback mask */
  XCOMFooter footer;
} XCOMParEKF_FEEDBACK;

#define EKF_STATEFREEZE_MASK_POSITION 0x00000001
#define EKF_STATEFREEZE_MASK_VELOCITY 0x00000002
#define EKF_STATEFREEZE_MASK_ATTITUDE 0x00000004
#define EKF_STATEFREEZE_MASK_SENSORERORS 0x00000008
/**
 * This parameter contains the state freeze mask for realtime navigation.
 * Setting an entry to ’1’ freezes the state. Per default, no states in
 * realtime navigation are frozen, the mask equals ’0’.
 */
typedef struct __attribute__((__packed__)) {
  XCOMHeader header;
  XCOMParHeader param_header;
  uint32_t state_freeze_mask; /**< Bitmask for freezing states in realtime
                                 navigation */
  XCOMFooter footer;
} XCOMParEKF_STATEFREEZE;

#define EKF_ZARU_DISABLE 0 /**< Disable Zero Angular Rate Update */
#define EKF_ZARU_ENABLE 1  /**< Enable Zero Angular Rate Update */
/**
 * This parameter contains configures Zero Angular Rate Updates.
 * Zero Angular Rate updates can be used for Land Navigation if a standstill has
 * been detected. Drift of the heading solution even during longer standstill
 * periods can be reduced by several orders of magnitude by enabling this.
 */
typedef struct __attribute__((__packed__)) {
  XCOMHeader header;
  XCOMParHeader param_header;
  uint8_t enable;       /**< Enable/Ddisable Zero Angular Rate Update */
  uint8_t reserved1[3]; /**< Reserved for further use */
  XCOMFooter footer;
} XCOMParEKF_ZARU;

/**
 * This parameter defines the initial standard deviations and the process noise
 * model for the used inertial sensors.
 */
typedef struct {
  double root_noise_psd[3]; /**< Root Noise for x-, y- and z-axis in:
                               accelerometers: [m/s2/sqrt(Hz)]; gyroscopes:
                               [rad/s/sqrt(Hz)] */
  double offset_stddev[3]; /**< Bias standard deviation for x-, y- and z-axis in
                              accelerometers: [m/s2]; gyroscopes: [rad/s] */
  double offset_rw[3];     /**< Bias random walk for x-, y- and z-axis in
                              accelerometers: [m/s3/sqrt(Hz)] gyroscopes:
                              [rad/s2/sqrt(Hz)] */
  double scalefactor_stddev[3]; /**< Scale factor standard deviation for x-, y-
                                   and z-axis */
  double scalefactor_rw[3]; /**< Scale factor random walk for x-, y- and z-axis
                               in: accelerometers: [1/s/sqrt(Hz)]; gyroscopes:
                               [1/s/sqrt(Hz)] */
  double ma_stddev; /**< Misalignment standard deviation in: accelerometers:
                       [m]; gyroscopes: [rad] */
  double ma_rw;     /**< Misalignment random walk in: accelerometers:
                       [m/s2/sqrt(Hz)]; gyroscopes: [rad/s/ Hz] */
  double quantization[3]; /**< Quantization for x-, y- and z-axis */
} XCOMparEKF_IMUCONFIGtype;

typedef struct __attribute__((__packed__)) {
  XCOMHeader header;
  uint16_t param_id;
  uint8_t estimation_config; /**< Estimated sensor misalignment configuration */
  uint8_t is_request;
  XCOMparEKF_IMUCONFIGtype
      acc; /**< Deviation values related to accelerometers */
  XCOMparEKF_IMUCONFIGtype
      gyro; /**< Deviation values related to angular rate sensors */
  XCOMFooter footer;
} XCOMParEKF_IMUCONFIG;

/**
 * This parameter defines the calibration time for the Zero Velocity Update
 * mechanism. The ZUPT calibration mechanism can be triggered via the
 * ZUPTCALIBRATION command.
 */
typedef struct __attribute__((__packed__)) {
  XCOMHeader header;
  XCOMParHeader param_header;
  uint16_t calib_time; /**< ZUPT calibration time in [s] */
  uint16_t reserved1;
  XCOMFooter footer;
} XCOMParEKF_ZUPTCALIB;

#define EKF_RECOVERY_MASK_ACCEL 0x00000001 /**< Accelerometer overrange */
/**
 * This parameter contains the realignment condition bit mask.
 * Setting an entry to ’1’ en- ables a condition in the automatic realignment
 * state machine. Per default, the realignment state machine is inactive and the
 * the mask equals ’0’.
 */
typedef struct __attribute__((__packed__)) {
  XCOMHeader header;
  XCOMParHeader param_header;
  uint32_t recovery_mask; /**< Bitmask for realignment conditions */
  XCOMFooter footer;
} XCOMParEKF_RECOVERY;

#define EKF_ODOCHECK_DISABLE                                                   \
  0                           /**< Disable odometer outlier detection module   \
                               */
#define EKF_ODOCHECK_ENABLE 1 /**< Enable odometer outlier detection module */
/**
 * This parameter configures the odometer outlier detection module
 */
typedef struct __attribute__((__packed__)) {
  XCOMHeader header;
  XCOMParHeader param_header;
  uint32_t enable;    /**< Enable/disable odometer outlier detection module */
  float vel_gnss_thr; /**< GNSS velocity threshold in [m/s] */
  float vel_odo_thr;  /**< Odometer velocity threshold in [m/s] */
  int max_error_gnss; /**< Allowed GNSS errors */
  float azi_interval; /**< Azimuth update interval in [sec] */
  float azi_limit;    /**< Allowed azimuth drift in [rad] */
  XCOMFooter footer;
} XCOMParEKF_ODOCHECK;

typedef struct __attribute__((__packed__)) {
  XCOMHeader header;
  XCOMParHeader param_header;
  uint16_t position_mode;
  uint16_t altitude_mode;
  XCOMFooter footer;
} XCOMParDAT_POS;

typedef struct __attribute__((__packed__)) {
  XCOMHeader header;
  XCOMParHeader param_header;
  uint32_t mode;
  XCOMFooter footer;
} XCOMParDAT_VEL;

typedef struct __attribute__((__packed__)) {
  XCOMHeader header;
  XCOMParHeader param_header;
  uint32_t mode;
  XCOMFooter footer;
} XCOMParDAT_IMU;

typedef struct __attribute__((__packed__)) {
  XCOMHeader header;
  XCOMParHeader param_header;
  uint32_t mode;
  XCOMFooter footer;
} XCOMParDAT_SYSSTAT;

typedef struct __attribute__((__packed__)) {
  XCOMHeader header;
  uint16_t param_id;
  uint8_t reserved;
  uint8_t is_request;
  uint32_t power_status_min;
  uint32_t power_status_max;
  uint16_t fpga_status;
  uint16_t supervisor_status;
  uint8_t imu_status;
  uint8_t temperature_status;
  uint16_t reserved1;
  XCOMFooter footer;
} t_XCOM_FPGASTATUS;

typedef struct __attribute__((__packed__)) {
  XCOMHeader header;
  uint32_t uiStatus;
  XCOMFooter footer;
} t_XCOM_ARINC429STATUS;

typedef struct __attribute__((__packed__)) {
  XCOMHeader header;
  uint16_t param_id;
  uint8_t ucReserved;
  uint8_t is_request;
  uint8_t ucPort;
  uint8_t ucSwitch;
  uint16_t reserved;
  uint32_t uiBaudRate;
  XCOMFooter footer;
} t_XCOM_PARXCOM_SERIALPORT;

typedef struct __attribute__((__packed__)) {
  XCOMHeader header;
  uint16_t param_id;
  uint8_t reserved;
  uint8_t is_request;
  uint8_t mode;
  uint8_t reserved_1;
  uint8_t interface;
  uint8_t reserved_2;
  uint32_t port;
  uint32_t ip_addr;
  uint32_t subnet_mask;
  uint32_t gateway;
  XCOMFooter footer;
} XCOMParXCOM_NETCONFIG;

typedef struct __attribute__((__packed__)) {
  XCOMHeader header;
  uint16_t param_id;
  uint8_t reserved;
  uint8_t is_request;
  uint32_t enable;
  XCOMFooter footer;
} XCOMParXCOM_NETCHECK;

typedef struct __attribute__((__packed__)) {
  XCOMHeader header;
  uint16_t param_id;
  uint8_t ucReserved;
  uint8_t is_request;
  struct {
    uint16_t divider;
    uint16_t msgID;
  } LogList[16];
  XCOMFooter footer;
} t_XCOM_PARXCOM_LOGLIST;

typedef struct __attribute__((__packed__)) {
  XCOMHeader header;
  uint16_t param_id;
  uint8_t channel;
  uint8_t is_request;
  struct {
    uint16_t divider;
    uint16_t msg_id;
    uint8_t running;
    uint8_t trigger;
    uint16_t reserved1;
  } log_list[16];
  XCOMFooter footer;
} XCOMParXCOM_LOGLIST2;

#define PARXCOM_INTERFACE_MODE_NONE 0
#define PARXCOM_INTERFACE_MODE_XCOM 1
#define PARXCOM_INTERFACE_MODE_XCOMAUTOSTART 2
#define PARXCOM_INTERFACE_MODE_PASSTHROUGH 3
#define PARXCOM_INTERFACE_MODE_GNSSPASSTHROUGH 4
#define PARXCOM_INTERFACE_MODE_GNSSCORRECTION 5

#define PARXCOM_INTERFACE_LEVEL_RS232 0
#define PARXCOM_INTERFACE_LEVEL_RS422 1
typedef struct __attribute__((__packed__)) {
  XCOMHeader header;
  uint16_t param_id;
  uint8_t reserved;
  uint8_t is_request;
  uint8_t port;
  uint8_t port_mode;
  uint8_t port_level;
  uint8_t available;
  uint32_t baud;
  uint8_t reserved1[8];
  XCOMFooter footer;
} XCOMParXCOM_INTERFACE;

typedef struct __attribute__((__packed__)) {
  XCOMHeader header;
  uint16_t param_id;
  uint8_t reserved;
  uint8_t is_request;
  uint16_t channel_number;
  uint16_t auto_start;
  uint16_t port;
  uint16_t reserved2;
  XCOMFooter footer;
} XCOMParXCOM_AUTOSTART;

typedef struct __attribute__((__packed__)) {
  XCOMHeader header;
  uint16_t param_id;
  uint8_t reserved;
  uint8_t is_request;
  char mount_point[128];
  char user_name[128];
  char password[128];
  char server[128];
  uint8_t send_position_on_login;
  uint8_t enable;
  uint16_t port;
  uint32_t gga_send_period;
  XCOMFooter footer;
} t_XCOM_PARXCOM_NTRIP;

typedef struct __attribute__((__packed__)) {
  XCOMHeader header;
  uint16_t param_id;
  uint8_t reserved;
  uint8_t is_request;
  uint8_t enable;
  uint8_t channel;
  uint16_t gga_send_period; /**< GGA send period in [sec] */
  XCOMFooter footer;
} XCOMParXCOM_POSTPROC;

typedef struct __attribute__((__packed__)) {
  XCOMHeader header;
  uint16_t param_id;
  uint8_t enable_inssol;
  uint8_t is_request;
  uint8_t enable;
  uint8_t channel;
  uint16_t divider;
  char path_name[256];
  XCOMFooter footer;
} XCOMParXCOM_CALPROC;

typedef struct __attribute__((__packed__)) {
  XCOMHeader header;
  uint16_t param_id;
  uint8_t ucReserved0;
  uint8_t is_request;
  uint32_t uiPort;
  uint8_t ucHiddenMode;
  uint8_t ucReserved1;
  uint16_t usReserved;
  XCOMFooter footer;
} t_XCOM_PARXCOM_BROADCAST;

typedef struct __attribute__((__packed__)) {
  XCOMHeader header;
  uint16_t param_id;
  uint8_t reserved;
  uint8_t is_request;
  uint32_t srv_addr;
  uint32_t port;
  uint8_t enable;
  uint8_t xcom_channel;
  uint8_t enable_abd;
  uint8_t instance;
  XCOMFooter footer;
} XCOMParXCOM_UDPCONFIG;

typedef struct __attribute__((__packed__)) {
  XCOMHeader header;
  uint16_t param_id;
  uint8_t reserved;
  uint8_t is_request;
  uint32_t log_level;
  uint32_t activation_mask;
  XCOMFooter footer;
} XCOMParXCOM_MONITOR;

typedef struct __attribute__((__packed__)) {
  XCOMHeader header;
  uint16_t param_id;
  uint8_t reserved;
  uint8_t is_request;
  uint32_t enable;
  XCOMFooter footer;
} XCOMParXCOM_QCONN;

typedef struct __attribute__((__packed__)) {
  XCOMHeader header;
  uint16_t param_id;
  uint8_t ucReserved0;
  uint8_t is_request;
  uint32_t uiEnable;
  XCOMFooter footer;
} t_XCOM_PARXCOM_DUMPENABLE;

typedef struct __attribute__((__packed__)) {
  XCOMHeader header;
  uint16_t param_id;
  uint8_t ucReserved0;
  uint8_t is_request;
  uint32_t uiEnable;
  XCOMFooter footer;
} t_XCOM_PARXCOM_MIGRATOR;

typedef struct __attribute__((__packed__)) {
  XCOMHeader header;
  uint16_t param_id;
  uint8_t ucReserved;
  uint8_t is_request;
  uint16_t usTimeout;
  uint16_t usInterval;
  uint16_t usProbes;
  uint16_t usEnable;
  XCOMFooter footer;
} t_XCOM_PARXCOM_TCPKEEPAL;

typedef struct __attribute__((__packed__)) {
  XCOMHeader header;
  uint16_t param_id;
  uint8_t ucReserved;
  uint8_t is_request;
  uint32_t uiSrcPort;
  uint32_t uiDestPort;
  uint32_t uiDestAddr;
  uint8_t ucARINC825loopback;
  uint8_t ucDebugEnable;
  uint8_t ucEnable;
  uint8_t ucInterface;
  XCOMFooter footer;
} t_XCOM_PARXCOM_CANGATEWAY;

typedef struct __attribute__((__packed__)) {
  XCOMHeader header;
  uint16_t param_id;
  uint8_t reserved;
  uint8_t is_request;
  uint32_t ip_addr;
  XCOMFooter footer;
} XCOMParXCOM_DEFAULTIP;

typedef struct __attribute__((__packed__)) {
  XCOMHeader header;
  uint16_t param_id;
  uint8_t ucReserved;
  uint8_t is_request;
  uint32_t dstPort;
  uint32_t srcPort;
  XCOMFooter footer;
} t_XCOM_PARXCOM_ABDCONFIG;

typedef struct __attribute__((__packed__)) {
  XCOMHeader header;
  uint16_t param_id;
  uint8_t reserved;
  uint8_t is_request;
  uint32_t output_frame;
  XCOMFooter footer;
} XCOMParXCOM_FRAMEOUT;

typedef struct __attribute__((__packed__)) {
  XCOMHeader header;
  uint16_t param_id;
  uint8_t ucEntries;
  uint8_t is_request;
  struct {
    uint32_t uiIpAddr;
    uint32_t uiPort;
    uint8_t ucEnable;
    uint8_t ucChannel;
    uint16_t usConnectionRetries;
    struct {
      uint8_t msgID;
      uint8_t trigger;
      uint16_t divider;
    } LOGS[XCOM_MAX_CLIENT_LOGS];
  } CONF[XCOM_MAX_CLIENT_SUPPORT];
  uint8_t useUDPinterface;
  uint8_t reserved[3];
  XCOMFooter footer;
} t_XCOM_PARXCOM_CLIENT;

typedef struct __attribute__((__packed__)) {
  XCOMHeader header;
  uint16_t param_id;
  uint8_t ucReserved;
  uint8_t is_request;
  char acTcpDumpArgs[256];
  XCOMFooter footer;
} t_XCOM_PARXCOM_TCPDUMP;

typedef struct __attribute__((__packed__)) {
  XCOMHeader header;
  uint16_t param_id;
  uint8_t reserved;
  uint8_t is_request;
  uint8_t reg;
  uint8_t reserved1;
  uint16_t password;
  XCOMFooter footer;
} XCOMParFPGA_8BITREGISTER;

typedef struct __attribute__((__packed__)) {
  XCOMHeader header;
  uint16_t param_id;
  uint8_t reserved;
  uint8_t is_request;
  uint8_t mode;
  uint8_t clock;
  uint8_t invert_data;
  uint8_t invert_clock;
  uint16_t reserved1[2];
  XCOMFooter footer;
} XCOMParFPGA_HDLC;

typedef struct __attribute__((__packed__)) {
  XCOMHeader header;
  uint16_t param_id;
  uint8_t reserved;
  uint8_t is_request;
  uint8_t timing_register;
  uint8_t reserved1;
  uint16_t user_timer[3];
  uint16_t reserved2;
  uint16_t password;
  XCOMFooter footer;
} XCOMParFPGA_TIMING;

#define FPGA_TIMERMATRIX_SIZE 32
typedef struct __attribute__((__packed__)) {
  XCOMHeader header;
  uint16_t param_id;
  uint8_t reserved;
  uint8_t is_request;
  uint32_t timer[FPGA_TIMERMATRIX_SIZE];
  uint16_t reserved1;
  uint16_t password;
  XCOMFooter footer;
} XCOMParFPGA_TIMER;

typedef struct __attribute__((__packed__)) {
  XCOMHeader header;
  uint16_t param_id;
  uint8_t reserved;
  uint8_t is_request;
  uint32_t interface[23];
  uint16_t reserved1;
  uint16_t password;
  XCOMFooter footer;
} XCOMParFPGA_INTERFACE;

typedef struct __attribute__((__packed__)) {
  XCOMHeader header;
  uint16_t param_id;
  uint8_t reserved;
  uint8_t is_request;
  uint16_t control_register;
  uint16_t reserved1;
  XCOMFooter footer;
} XCOMParFPGA_CONTROLREG;

typedef struct __attribute__((__packed__)) {
  XCOMHeader header;
  uint16_t param_id;
  uint8_t reserved;
  uint8_t is_request;
  uint16_t trigger_timeout;
  uint16_t reserved1;
  XCOMFooter footer;
} XCOMParFPGA_TRIGTIMEOUT;

typedef struct __attribute__((__packed__)) {
  XCOMHeader header;
  uint16_t param_id;
  uint8_t reserved;
  uint8_t is_request;
  float threshold;
  float reserved1;
  XCOMFooter footer;
} XCOMParFPGA_POWERUPTHR;

typedef struct __attribute__((__packed__)) {
  XCOMHeader header;
  uint16_t param_id;
  uint8_t reserved;
  uint8_t is_request;
  float holdover_time; // in [s]
  XCOMFooter footer;
} XCOMParFPGA_HOLDOVERTIME;

typedef struct __attribute__((__packed__)) {
  XCOMHeader header;
  uint16_t param_id;
  uint8_t ucReserved;
  uint8_t is_request;
  uint8_t aInterfaceMatrix[32];
  XCOMFooter footer;
} t_XCOM_PARFPGA_INTMAT245;

/* FPGA Type */
#define XCOM_PARFPGA_TYPE_NONE 0
#define XCOM_PARFPGA_TYPE_288 1
#define XCOM_PARFPGA_TYPE_245 2
#define XCOM_PARFPGA_TYPE_CFM 3
#define XCOM_PARFPGA_TYPE_288V3 4
#define XCOM_PARFPGA_TYPE_4CORE 5
#define XCOM_PARFPGA_TYPE_288V4 6
#define XCOM_PARFPGA_TYPE_M200OEMV4 7
typedef struct __attribute__((__packed__)) {
  XCOMHeader header;
  uint16_t param_id;
  uint8_t reserved;
  uint8_t is_request;
  uint32_t type;
  XCOMFooter footer;
} XCOMParFPGA_TYPE;

typedef struct __attribute__((__packed__)) {
  XCOMHeader header;
  uint16_t param_id;
  uint8_t ucPPTdivider;
  uint8_t is_request;
  uint32_t uiCONF;
  XCOMFooter footer;
} t_XCOM_PARFPGA_GLOBALCONF;

typedef struct __attribute__((__packed__)) {
  XCOMHeader header;
  uint16_t param_id;
  uint8_t ucReserved;
  uint8_t is_request;
  uint32_t uiPinMode;
  XCOMFooter footer;
} t_XCOM_PARFPGA_HDLCPINMODE;

typedef struct __attribute__((__packed__)) {
  XCOMHeader header;
  uint16_t param_id;
  uint8_t reserved;
  uint8_t is_request;
  uint32_t power_switches;
  XCOMFooter footer;
} XCOMParFPGA_POWER;

typedef struct __attribute__((__packed__)) {
  XCOMHeader header;
  uint16_t param_id;
  uint8_t ucReserved;
  uint8_t is_request;
  uint32_t uiAlarmThr;
  XCOMFooter footer;
} t_XCOM_PARFPGA_ALARMTHR;

typedef struct __attribute__((__packed__)) {
  XCOMHeader header;
  uint16_t param_id;
  uint8_t reserved;
  uint8_t is_request;
  uint16_t ppt_divider;
  uint8_t ppt_pulse_width;
  uint8_t reserved1;
  XCOMFooter footer;
} XCOMParFPGA_PPTCONFIG;

typedef struct __attribute__((__packed__)) {
  XCOMHeader header;
  uint16_t param_id;
  uint8_t reserved;
  uint8_t is_request;
  uint8_t enable_auto_wakeup;
  uint8_t interval; /**< [sec] */
  uint8_t retries;
  uint8_t reserved1;
  XCOMFooter footer;
} XCOMParFPGA_AUTOWAKEUP;

typedef struct __attribute__((__packed__)) {
  XCOMHeader header;
  uint16_t param_id;
  uint8_t reserved;
  uint8_t is_request;
  uint8_t gpio_reg;
  uint8_t reserved_1[3];
  XCOMFooter footer;
} XCOMParFPGA_MCP23S08;

typedef struct __attribute__((__packed__)) {
  XCOMHeader header;
  uint16_t param_id;
  uint8_t ucReserved;
  uint8_t is_request;
  uint8_t mode;

  uint32_t PPSdisciplineTimeConstant;
  uint32_t PPSdisciplineCableLengthComp;
  XCOMFooter footer;
} t_XCOM_PARFPGA_CSAC;

typedef struct __attribute__((__packed__)) {
  XCOMHeader header;
  uint16_t param_id;
  uint8_t reserved;
  uint8_t is_request;
  uint32_t mask;
  XCOMFooter footer;
} XCOMParFPGA_32BITMASK;

// typedef struct __attribute__ ((__packed__))
//{
//    XCOMHeader header;
//    uint16_t      param_id;
//    uint8_t	  	  ucReserved;
//    uint8_t	 	  is_request;
//    uint8_t       ucPort;
//    uint8_t       ucEnableUART;
//    uint16_t      usReserved;
//    uint32_t      uiBaudRate;
//    XCOMFooter footer;
//} t_XCOM_PARNMEA_COM;

// typedef struct __attribute__ ((__packed__))
//{
//    XCOMHeader header;
//    uint16_t      param_id;
//    uint8_t	  	  ucReserved0;
//    uint8_t	 	  is_request;
//    uint8_t       ucReserved1;
//    uint8_t       ucGpsQualityMode;
//    uint8_t       ucSelectionSwitch;
//    uint8_t       ucReserved2;
//    XCOMFooter footer;
//} t_XCOM_PARNMEA_ENABLE;

// typedef struct __attribute__ ((__packed__))
//{
//    XCOMHeader header;
//    uint16_t      param_id;
//    uint8_t	  	  ucReserved;
//    uint8_t	 	  is_request;
//    uint32_t      uiTxMask;
//    XCOMFooter footer;
//} t_XCOM_PARNMEA_TXMASK;

// typedef struct __attribute__ ((__packed__))
//{
//    XCOMHeader header;
//    uint16_t      param_id;
//    uint8_t	  	  ucReserved;
//    uint8_t	 	  is_request;
//    uint8_t       ucDecPlacesPos;
//    uint8_t       ucDecPlacesHdg;
//    uint16_t      usReserved;
//    XCOMFooter footer;
//} t_XCOM_PARNMEA_DECPLACES;

// typedef struct __attribute__ ((__packed__))
//{
//    XCOMHeader header;
//    uint16_t      param_id;
//    uint8_t	  	  ucReserved;
//    uint8_t	 	  is_request;
//    uint32_t	  uiDivisor;
//    XCOMFooter footer;
//} t_XCOM_PARNMEA_RATE;

// typedef struct __attribute__ ((__packed__))
//{
//    XCOMHeader header;
//    uint16_t      param_id;
//    uint8_t	  	  ucReserved;
//    uint8_t	 	  is_request;
//    uint32_t      uiSrvAddr;
//    uint32_t      uiPort;
//    uint8_t       ucEnable;
//    uint8_t       ucRes;
//    uint16_t	  usReserved;
//    XCOMFooter footer;
//} t_XCOM_PARNMEA_UDP;

struct __attribute__((__packed__)) NMEA_MSG_CONF {
  uint8_t msgId;
  uint8_t src; ///< INS/GNSS
  int divider; ///< divider == -1: pps-triggered
  uint8_t useIMARGPSQI;
};

struct __attribute__((__packed__)) NMEA_COM_CONF {
  uint32_t port;
  uint32_t baud;
  uint8_t enable;
  uint8_t numOfMsg;
  struct NMEA_MSG_CONF msgConf[NMEA_MAX_MSG_COM];
};

typedef struct __attribute__((__packed__)) {
  XCOMHeader header;
  uint16_t param_id;
  uint8_t ucReserved;
  uint8_t is_request;

  uint8_t numOfCom;
  struct NMEA_COM_CONF comConf[NMEA_MAX_COM];

  uint16_t usReserved;
  XCOMFooter footer;
} t_XCOMPAR_PARNMEA_COM;

struct __attribute__((__packed__)) NMEA_UDP_CONF {
  uint32_t ipAddr;
  uint32_t port;
  uint8_t enable;
  uint8_t numOfMsg;
  struct NMEA_MSG_CONF msgConf[NMEA_MAX_MSG_UDP];
};

typedef struct __attribute__((__packed__)) {
  XCOMHeader header;
  uint16_t param_id;
  uint8_t ucReserved;
  uint8_t is_request;

  uint8_t numOfUdp;
  struct NMEA_UDP_CONF udpConf[NMEA_MAX_UDP];

  uint16_t usReserved;
  XCOMFooter footer;
} t_XCOMPAR_PARNMEA_UDP;

typedef struct __attribute__((__packed__)) {
  XCOMHeader header;
  uint16_t param_id;
  uint8_t ucReserved;
  uint8_t is_request;
  uint8_t ucPort;
  uint8_t ucEnable;
  uint16_t usReserved0;
  uint8_t ucRes1;
  uint8_t ucHighSpeed;
  uint16_t usReserved1;
  XCOMFooter footer;
} t_XCOM_PARARINC429_CONFIG;

typedef struct __attribute__((__packed__)) {
  uint8_t m_channel;
  uint8_t m_label;
  uint8_t m_datIdx;
  uint8_t m_enable;
  double m_range;
  double m_scf;
  uint8_t m_width;
  uint32_t m_period;
  uint32_t m_timer;
} t_XCOM_PARARINC429_LISTITEM;

typedef struct __attribute__((__packed__)) {
  XCOMHeader header;
  uint16_t param_id;
  uint8_t ucReserved;
  uint8_t is_request;
  t_XCOM_PARARINC429_LISTITEM tLIST[32];
  XCOMFooter footer;
} t_XCOM_PARARINC429_LIST;

typedef struct __attribute__((__packed__)) {
  XCOMHeader header;
  uint16_t param_id;
  uint8_t pptDivider;
  uint8_t is_request;
  uint32_t configIO;
  XCOMFooter footer;
} t_XCOM_PARIO_HW245;

/* User specific logs */
typedef struct __attribute__((__packed__)) {
  XCOMHeader header;
  float accel[3];
  float gyro[3];
  float rpy[3];
  float vel[3];
  double pos[2];
  float altitude;
  uint16_t usDiffAge;
  uint16_t usReserved;
  XCOMFooter footer;
} t_XCOM_USR_INSSOL;

typedef struct __attribute__((__packed__)) {
  XCOMHeader header;
  float accel[3];
  float gyro[3];
  float rpy[3];
  float fTempIMU;
  float vel[3];
  int32_t iLon;
  int32_t iLat;
  float fAlt;
  uint32_t uiStatus;
  uint16_t usSOG;
  uint16_t usCRC;
} t_XCOM_USR_THALESHS;

typedef struct __attribute__((__packed__)) {
  XCOMHeader header;
  uint32_t uiTOW;
  int16_t sWeek;
  uint8_t ucGpsFix;
  uint8_t ucGpsNavFlags;
  int32_t iLon;
  int32_t iLat;
  int32_t iHeight;
  int32_t iVnorth;
  int32_t iVeast;
  int32_t iVdown;
  int32_t iCOG;
  uint8_t ucSV;
  uint8_t ucReserved;
  uint16_t usPDOP;
  uint16_t usReserved;
  uint16_t usCRC;
} t_XCOM_USR_THALESLS;

typedef struct __attribute__((__packed__)) {
  XCOMHeader header;
  float fBias[6];
  float fCov[15];
  uint16_t usReserved;
  uint16_t usCRC;
} t_XCOM_USR_THALESCOV;

typedef struct __attribute__((__packed__)) {
  XCOMHeader header;
  float fIAS;
  float fTAS;
  float fPs;
  float fAlt;
  float fOAT;
  float fWindSpeed;
  float fWindDir;
  float fMagDev;
  uint16_t usStatus;
  uint16_t usCRC;
} t_XCOM_USR_THALESAIR;

typedef struct __attribute__((__packed__)) {
  XCOMHeader header;
  float gyro[3]; // IMUCORR
  float fQUAT[4];
  float vel[3];
  double pos[3];
  XCOMFooter footer;
} t_XCOM_USR_INSSOL_SCU;

typedef struct __attribute__((__packed__)) {
  XCOMHeader header;
  float gyro[3]; // IMUCORR
  float fQUAT[4];
  XCOMFooter footer;
} t_XCOM_USR_INSSOL_SCU2;

typedef struct __attribute__((__packed__)) {
  XCOMHeader header;
  float fOmg[3]; // IMUCOMP
  float fRpy[3];
  float fAcc[3];
  float fVb[3];
  float fVecef[3];
  double dLongitude;
  double dLatitude;
  double dPosECEF[3]; // x,y,z
  float fDCM[9];
  XCOMFooter footer;
} t_XCOM_USR_ADSE;

typedef struct __attribute__((__packed__)) {
  XCOMHeader header;
  double txTime; /**< GPS time */
  double posECEF[3];
  XCOMFooter tBottom;
} t_XCOM_USR_LATENCYTX;

typedef struct __attribute__((__packed__)) {
  XCOMHeader header;
  int slave_idx;
  double slave_gpsTimeTx;  /**< Transmitter 	*/
  double slave_posECEF[3]; /**< Transmitter 	*/
  double slave_gpsTOV;     /**< Transmitter 	*/
  /* -------------------------------------------------------- */
  double master_gpsTimeRx;  /**< Receiver  	*/
  double master_posECEF[3]; /**< Receiver 	*/
  double master_gpsTOV;     /**< Receiver 	*/
  XCOMFooter footer;
} t_XCOM_USR_LATENCYRX;

#endif /* XCOMDAT_H_ */
