/** THIS IS AN AUTOMATICALLY GENERATED FILE.  DO NOT MODIFY
 * BY HAND!!
 *
 * Generated by zcm-gen
 **/

#include <zcm/zcm_coretypes.h>

#ifndef __line_t_hpp__
#define __line_t_hpp__



/// zcmtypes/line_t.hpp
class line_t
{
    public:
        double     line_coords[4];

        int8_t     color;

        int8_t     cameraId;

    public:
        #if __cplusplus > 199711L /* if c++11 */
        static constexpr int8_t   X1 = 0;
        static constexpr int8_t   Y1 = 1;
        static constexpr int8_t   X2 = 2;
        static constexpr int8_t   Y2 = 3;
        static constexpr int8_t   WHITE = 0;
        static constexpr int8_t   GREEN = 1;
        static constexpr int8_t   RED = 2;
        static constexpr int8_t   BOTTOM_CAM = 0;
        static constexpr int8_t   FRONT_CAM = 1;
        static constexpr int8_t   RIGHT_CAM = 2;
        static constexpr int8_t   BACK_CAM = 3;
        static constexpr int8_t   LEFT_CAM = 4;
        #else
        static const     int8_t   X1 = 0;
        static const     int8_t   Y1 = 1;
        static const     int8_t   X2 = 2;
        static const     int8_t   Y2 = 3;
        static const     int8_t   WHITE = 0;
        static const     int8_t   GREEN = 1;
        static const     int8_t   RED = 2;
        static const     int8_t   BOTTOM_CAM = 0;
        static const     int8_t   FRONT_CAM = 1;
        static const     int8_t   RIGHT_CAM = 2;
        static const     int8_t   BACK_CAM = 3;
        static const     int8_t   LEFT_CAM = 4;
        #endif

    public:
        /**
         * Destructs a message properly if anything inherits from it
        */
        virtual ~line_t() {}

        /**
         * Encode a message into binary form.
         *
         * @param buf The output buffer.
         * @param offset Encoding starts at thie byte offset into @p buf.
         * @param maxlen Maximum number of bytes to write.  This should generally be
         *  equal to getEncodedSize().
         * @return The number of bytes encoded, or <0 on error.
         */
        inline int encode(void *buf, int offset, int maxlen) const;

        /**
         * Check how many bytes are required to encode this message.
         */
        inline int getEncodedSize() const;

        /**
         * Decode a message from binary form into this instance.
         *
         * @param buf The buffer containing the encoded message.
         * @param offset The byte offset into @p buf where the encoded message starts.
         * @param maxlen The maximum number of bytes to reqad while decoding.
         * @return The number of bytes decoded, or <0 if an error occured.
         */
        inline int decode(const void *buf, int offset, int maxlen);

        /**
         * Retrieve the 64-bit fingerprint identifying the structure of the message.
         * Note that the fingerprint is the same for all instances of the same
         * message type, and is a fingerprint on the message type definition, not on
         * the message contents.
         */
        inline static int64_t getHash();

        /**
         * Returns "line_t"
         */
        inline static const char* getTypeName();

        // ZCM support functions. Users should not call these
        inline int _encodeNoHash(void *buf, int offset, int maxlen) const;
        inline int _getEncodedSizeNoHash() const;
        inline int _decodeNoHash(const void *buf, int offset, int maxlen);
        inline static uint64_t _computeHash(const __zcm_hash_ptr *p);
};

int line_t::encode(void *buf, int offset, int maxlen) const
{
    int pos = 0, tlen;
    int64_t hash = (int64_t)getHash();

    tlen = __int64_t_encode_array(buf, offset + pos, maxlen - pos, &hash, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = this->_encodeNoHash(buf, offset + pos, maxlen - pos);
    if (tlen < 0) return tlen; else pos += tlen;

    return pos;
}

int line_t::decode(const void *buf, int offset, int maxlen)
{
    int pos = 0, thislen;

    int64_t msg_hash;
    thislen = __int64_t_decode_array(buf, offset + pos, maxlen - pos, &msg_hash, 1);
    if (thislen < 0) return thislen; else pos += thislen;
    if (msg_hash != getHash()) return -1;

    thislen = this->_decodeNoHash(buf, offset + pos, maxlen - pos);
    if (thislen < 0) return thislen; else pos += thislen;

    return pos;
}

int line_t::getEncodedSize() const
{
    return 8 + _getEncodedSizeNoHash();
}

int64_t line_t::getHash()
{
    static int64_t hash = _computeHash(NULL);
    return hash;
}

const char* line_t::getTypeName()
{
    return "line_t";
}

int line_t::_encodeNoHash(void *buf, int offset, int maxlen) const
{
    int pos = 0, tlen;

    tlen = __double_encode_array(buf, offset + pos, maxlen - pos, &this->line_coords[0], 4);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __int8_t_encode_array(buf, offset + pos, maxlen - pos, &this->color, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __int8_t_encode_array(buf, offset + pos, maxlen - pos, &this->cameraId, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    return pos;
}

int line_t::_decodeNoHash(const void *buf, int offset, int maxlen)
{
    int pos = 0, tlen;

    tlen = __double_decode_array(buf, offset + pos, maxlen - pos, &this->line_coords[0], 4);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __int8_t_decode_array(buf, offset + pos, maxlen - pos, &this->color, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __int8_t_decode_array(buf, offset + pos, maxlen - pos, &this->cameraId, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    return pos;
}

int line_t::_getEncodedSizeNoHash() const
{
    int enc_size = 0;
    enc_size += __double_encoded_array_size(NULL, 4);
    enc_size += __int8_t_encoded_array_size(NULL, 1);
    enc_size += __int8_t_encoded_array_size(NULL, 1);
    return enc_size;
}

uint64_t line_t::_computeHash(const __zcm_hash_ptr *)
{
    uint64_t hash = (uint64_t)0x2767b73bccd02913LL;
    return (hash<<1) + ((hash>>63)&1);
}

#endif
