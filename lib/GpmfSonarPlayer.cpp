
#include <fstream>

#include "Oculus/Oculus.h"

#include "g3log/g3log.hpp"

#include "liboculus/DataTypes.h"
#include "liboculus/SimplePingResult.h"
#include "liboculus/GpmfSonarPlayer.h"

namespace liboculus {

  //--- GPMFSonarPlayer --

  GPMFSonarPlayer::GPMFSonarPlayer()
    : SonarPlayerBase(),
      _stream(),
      _valid(false),
      _buffer()
    {;}

  GPMFSonarPlayer::~GPMFSonarPlayer()
    {;}

     bool GPMFSonarPlayer::open( const std::string &filename ) {
       {
         bool retval = SonarPlayerBase::open( filename );
        if( !retval ) return retval;
       }

      _input.seekg(0, std::ios::end);
      const size_t sz = _input.tellg();
      _buffer.resize( sz, '\0' );
      _input.seekg(0, std::ios::beg);

      _input.read( &_buffer[0], sz );

      // _buffer.assign((std::istreambuf_iterator<char>(_input)),
      //             std::istreambuf_iterator<char>());

      LOG(DEBUG) << "Loading " << _buffer.size() << " bytes";

      GPMF_Init( &_stream, (unsigned int *)_buffer.c_str(), (_buffer.size()) );

      // {
      //   auto retval = GPMF_Validate(&_stream, GPMF_RECURSE_LEVELS);
      //   if( retval != GPMF_OK ) {
      //     LOG(WARNING) << "GPMF structure is not valid; err = " << retval;
      //     return false;
      //   }
      // }

     {
       auto retval = GPMF_FindNext( &_stream, STR2FOURCC("OCUS"), GPMF_RECURSE_LEVELS );

       if( retval != GPMF_OK ) {
         LOG(INFO) << "Unable to find Oculus sonar data in GPMF file (err " << retval << ")";
         return false;
       }
     }

       return true;
      }

     bool GPMFSonarPlayer::eof() {
       // How to handle this?
      auto retval = GPMF_Validate(&_stream, GPMF_RECURSE_LEVELS);
      return retval == GPMF_ERROR_BUFFER_END;
     }

     void GPMFSonarPlayer::rewind() {
       if( _valid ) {
         GPMF_ResetState( &_stream );
       }
     }

     void GPMFSonarPlayer::close() {
       _valid = false;
     }

     void GPMFSonarPlayer::dumpGPMF() {
       auto key = GPMF_Key( &_stream );
       LOG(INFO) << "Current key \"" << char((key >> 0)&0xFF) << char((key >> 8)&0xFF) << char((key>>16)&0xFF) << char((key>>24)&0xFF) << "\" (" << std::hex << key << ")";
       LOG(INFO) << "Current type " << GPMF_Type( &_stream );
       LOG(INFO) << "Current device ID " << std::hex << GPMF_DeviceID( &_stream );

       char deviceName[80];
       GPMF_DeviceName( &_stream, deviceName, 79 );
       LOG(INFO) << "Current device name " << deviceName;

       LOG(INFO) << "Current struct size " << GPMF_StructSize( &_stream );
       LOG(INFO) << "Current repeat size " << GPMF_Repeat( &_stream );
       LOG(INFO) << "Current payload sample count " << GPMF_PayloadSampleCount( &_stream );
       LOG(INFO) << "Current elements in struct " << GPMF_ElementsInStruct( &_stream );
       LOG(INFO) << "Current raw data size " << GPMF_RawDataSize( &_stream );
     }

  std::shared_ptr<SimplePingResult> GPMFSonarPlayer::nextPing() {
    //
    // Ended up not needing to implement.  Oculus client records messagePingResult,
    // which we don't have the format for ...
    auto key = GPMF_Key( &_stream );
    if( key != STR2FOURCC("OCUS") ) return std::shared_ptr<SimplePingResult>(nullptr);

    shared_ptr<MessageBuffer> buffer( new MessageBuffer((char *)GPMF_RawData(&_stream), GPMF_RawDataSize( &_stream )) );
    // char *data = (char *)GPMF_RawData(&_stream);
    // CHECK(data != nullptr);

    MessageHeader header( buffer );
    if( !header.valid() ) {
      LOG(INFO) << "Invalid header";
      return std::shared_ptr<SimplePingResult>(nullptr);
    }

    auto retval = GPMF_FindNext( &_stream, STR2FOURCC("OCUS"), GPMF_RECURSE_LEVELS );
    if( retval != GPMF_OK ) {
      _valid = false;
    }

    return std::shared_ptr<SimplePingResult>( new SimplePingResult( buffer ) );
  }

}
