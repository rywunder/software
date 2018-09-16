/** THIS IS AN AUTOMATICALLY GENERATED FILE.  DO NOT MODIFY
 * BY HAND!!
 *
 * Generated by zcm-gen
 **/

#include <zcm/zcm_coretypes.h>

#ifndef __visual_data_t_hpp__
#define __visual_data_t_hpp__

#include <vector>
#include "line_t.hpp"
#include "visual_landmark_t.hpp"


/**
 * zcmtypes/visual_lines_t.hpp
 * ZCM message sent to localization
 */
class visual_data_t
{
    public:
        int64_t    utime;

        int16_t    num_lines;

        int16_t    num_landmarks;

        std::vector< line_t > lines;

        std::vector< visual_landmark_t > landmarks;

    public:
        /**
         * Destructs a message properly if anything inherits from it
        */
        virtual ~visual_data_t() {}

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
         * Returns "visual_data_t"
         */
        inline static const char* getTypeName();

        // ZCM support functions. Users should not call these
        inline int _encodeNoHash(void *buf, int offset, int maxlen) const;
        inline int _getEncodedSizeNoHash() const;
        inline int _decodeNoHash(const void *buf, int offset, int maxlen);
        inline static uint64_t _computeHash(const __zcm_hash_ptr *p);
};

int visual_data_t::encode(void *buf, int offset, int maxlen) const
{
    int pos = 0, tlen;
    int64_t hash = (int64_t)getHash();

    tlen = __int64_t_encode_array(buf, offset + pos, maxlen - pos, &hash, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = this->_encodeNoHash(buf, offset + pos, maxlen - pos);
    if (tlen < 0) return tlen; else pos += tlen;

    return pos;
}

int visual_data_t::decode(const void *buf, int offset, int maxlen)
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

int visual_data_t::getEncodedSize() const
{
    return 8 + _getEncodedSizeNoHash();
}

int64_t visual_data_t::getHash()
{
    static int64_t hash = _computeHash(NULL);
    return hash;
}

const char* visual_data_t::getTypeName()
{
    return "visual_data_t";
}

int visual_data_t::_encodeNoHash(void *buf, int offset, int maxlen) const
{
    int pos = 0, tlen;

    tlen = __int64_t_encode_array(buf, offset + pos, maxlen - pos, &this->utime, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __int16_t_encode_array(buf, offset + pos, maxlen - pos, &this->num_lines, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __int16_t_encode_array(buf, offset + pos, maxlen - pos, &this->num_landmarks, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    for (int a0 = 0; a0 < this->num_lines; a0++) {
        tlen = this->lines[a0]._encodeNoHash(buf, offset + pos, maxlen - pos);
        if(tlen < 0) return tlen; else pos += tlen;
    }

    for (int a0 = 0; a0 < this->num_landmarks; a0++) {
        tlen = this->landmarks[a0]._encodeNoHash(buf, offset + pos, maxlen - pos);
        if(tlen < 0) return tlen; else pos += tlen;
    }

    return pos;
}

int visual_data_t::_decodeNoHash(const void *buf, int offset, int maxlen)
{
    int pos = 0, tlen;

    tlen = __int64_t_decode_array(buf, offset + pos, maxlen - pos, &this->utime, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __int16_t_decode_array(buf, offset + pos, maxlen - pos, &this->num_lines, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __int16_t_decode_array(buf, offset + pos, maxlen - pos, &this->num_landmarks, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    this->lines.resize(this->num_lines);
    for (int a0 = 0; a0 < this->num_lines; a0++) {
        tlen = this->lines[a0]._decodeNoHash(buf, offset + pos, maxlen - pos);
        if(tlen < 0) return tlen; else pos += tlen;
    }

    this->landmarks.resize(this->num_landmarks);
    for (int a0 = 0; a0 < this->num_landmarks; a0++) {
        tlen = this->landmarks[a0]._decodeNoHash(buf, offset + pos, maxlen - pos);
        if(tlen < 0) return tlen; else pos += tlen;
    }

    return pos;
}

int visual_data_t::_getEncodedSizeNoHash() const
{
    int enc_size = 0;
    enc_size += __int64_t_encoded_array_size(NULL, 1);
    enc_size += __int16_t_encoded_array_size(NULL, 1);
    enc_size += __int16_t_encoded_array_size(NULL, 1);
    for (int a0 = 0; a0 < this->num_lines; a0++) {
        enc_size += this->lines[a0]._getEncodedSizeNoHash();
    }
    for (int a0 = 0; a0 < this->num_landmarks; a0++) {
        enc_size += this->landmarks[a0]._getEncodedSizeNoHash();
    }
    return enc_size;
}

uint64_t visual_data_t::_computeHash(const __zcm_hash_ptr *p)
{
    const __zcm_hash_ptr *fp;
    for(fp = p; fp != NULL; fp = fp->parent)
        if(fp->v == visual_data_t::getHash)
            return 0;
    const __zcm_hash_ptr cp = { p, (void*)visual_data_t::getHash };

    uint64_t hash = (uint64_t)0x7242853851c1fb69LL +
         line_t::_computeHash(&cp) +
         visual_landmark_t::_computeHash(&cp);

    return (hash<<1) + ((hash>>63)&1);
}

#endif
