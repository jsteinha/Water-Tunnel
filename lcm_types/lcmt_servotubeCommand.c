/** THIS IS AN AUTOMATICALLY GENERATED FILE.  DO NOT MODIFY
 * BY HAND!!
 *
 * Generated by lcm-gen
 **/

#include <string.h>
#include "lcmt_servotubeCommand.h"

static int __lcmt_servotubeCommand_hash_computed;
static int64_t __lcmt_servotubeCommand_hash;
 
int64_t __lcmt_servotubeCommand_hash_recursive(const __lcm_hash_ptr *p)
{
    const __lcm_hash_ptr *fp;
    for (fp = p; fp != NULL; fp = fp->parent)
        if (fp->v == __lcmt_servotubeCommand_get_hash)
            return 0;
 
    const __lcm_hash_ptr cp = { p, (void*)__lcmt_servotubeCommand_get_hash };
    (void) cp;
 
    int64_t hash = 0xebcdb7e5120c9d50LL
         + __int64_t_hash_recursive(&cp)
         + __double_hash_recursive(&cp)
         + __int32_t_hash_recursive(&cp)
        ;
 
    return (hash<<1) + ((hash>>63)&1);
}
 
int64_t __lcmt_servotubeCommand_get_hash(void)
{
    if (!__lcmt_servotubeCommand_hash_computed) {
        __lcmt_servotubeCommand_hash = __lcmt_servotubeCommand_hash_recursive(NULL);
        __lcmt_servotubeCommand_hash_computed = 1;
    }
 
    return __lcmt_servotubeCommand_hash;
}
 
int __lcmt_servotubeCommand_encode_array(void *buf, int offset, int maxlen, const lcmt_servotubeCommand *p, int elements)
{
    int pos = 0, thislen, element;
 
    for (element = 0; element < elements; element++) {
 
        thislen = __int64_t_encode_array(buf, offset + pos, maxlen - pos, &(p[element].timestamp), 1);
        if (thislen < 0) return thislen; else pos += thislen;
 
        thislen = __double_encode_array(buf, offset + pos, maxlen - pos, &(p[element].commandValue), 1);
        if (thislen < 0) return thislen; else pos += thislen;
 
        thislen = __int32_t_encode_array(buf, offset + pos, maxlen - pos, &(p[element].commandType), 1);
        if (thislen < 0) return thislen; else pos += thislen;
 
    }
    return pos;
}
 
int lcmt_servotubeCommand_encode(void *buf, int offset, int maxlen, const lcmt_servotubeCommand *p)
{
    int pos = 0, thislen;
    int64_t hash = __lcmt_servotubeCommand_get_hash();
 
    thislen = __int64_t_encode_array(buf, offset + pos, maxlen - pos, &hash, 1);
    if (thislen < 0) return thislen; else pos += thislen;
 
    thislen = __lcmt_servotubeCommand_encode_array(buf, offset + pos, maxlen - pos, p, 1);
    if (thislen < 0) return thislen; else pos += thislen;
 
    return pos;
}
 
int __lcmt_servotubeCommand_encoded_array_size(const lcmt_servotubeCommand *p, int elements)
{
    int size = 0, element;
    for (element = 0; element < elements; element++) {
 
        size += __int64_t_encoded_array_size(&(p[element].timestamp), 1);
 
        size += __double_encoded_array_size(&(p[element].commandValue), 1);
 
        size += __int32_t_encoded_array_size(&(p[element].commandType), 1);
 
    }
    return size;
}
 
int lcmt_servotubeCommand_encoded_size(const lcmt_servotubeCommand *p)
{
    return 8 + __lcmt_servotubeCommand_encoded_array_size(p, 1);
}
 
int __lcmt_servotubeCommand_decode_array(const void *buf, int offset, int maxlen, lcmt_servotubeCommand *p, int elements)
{
    int pos = 0, thislen, element;
 
    for (element = 0; element < elements; element++) {
 
        thislen = __int64_t_decode_array(buf, offset + pos, maxlen - pos, &(p[element].timestamp), 1);
        if (thislen < 0) return thislen; else pos += thislen;
 
        thislen = __double_decode_array(buf, offset + pos, maxlen - pos, &(p[element].commandValue), 1);
        if (thislen < 0) return thislen; else pos += thislen;
 
        thislen = __int32_t_decode_array(buf, offset + pos, maxlen - pos, &(p[element].commandType), 1);
        if (thislen < 0) return thislen; else pos += thislen;
 
    }
    return pos;
}
 
int __lcmt_servotubeCommand_decode_array_cleanup(lcmt_servotubeCommand *p, int elements)
{
    int element;
    for (element = 0; element < elements; element++) {
 
        __int64_t_decode_array_cleanup(&(p[element].timestamp), 1);
 
        __double_decode_array_cleanup(&(p[element].commandValue), 1);
 
        __int32_t_decode_array_cleanup(&(p[element].commandType), 1);
 
    }
    return 0;
}
 
int lcmt_servotubeCommand_decode(const void *buf, int offset, int maxlen, lcmt_servotubeCommand *p)
{
    int pos = 0, thislen;
    int64_t hash = __lcmt_servotubeCommand_get_hash();
 
    int64_t this_hash;
    thislen = __int64_t_decode_array(buf, offset + pos, maxlen - pos, &this_hash, 1);
    if (thislen < 0) return thislen; else pos += thislen;
    if (this_hash != hash) return -1;
 
    thislen = __lcmt_servotubeCommand_decode_array(buf, offset + pos, maxlen - pos, p, 1);
    if (thislen < 0) return thislen; else pos += thislen;
 
    return pos;
}
 
int lcmt_servotubeCommand_decode_cleanup(lcmt_servotubeCommand *p)
{
    return __lcmt_servotubeCommand_decode_array_cleanup(p, 1);
}
 
int __lcmt_servotubeCommand_clone_array(const lcmt_servotubeCommand *p, lcmt_servotubeCommand *q, int elements)
{
    int element;
    for (element = 0; element < elements; element++) {
 
        __int64_t_clone_array(&(p[element].timestamp), &(q[element].timestamp), 1);
 
        __double_clone_array(&(p[element].commandValue), &(q[element].commandValue), 1);
 
        __int32_t_clone_array(&(p[element].commandType), &(q[element].commandType), 1);
 
    }
    return 0;
}
 
lcmt_servotubeCommand *lcmt_servotubeCommand_copy(const lcmt_servotubeCommand *p)
{
    lcmt_servotubeCommand *q = (lcmt_servotubeCommand*) malloc(sizeof(lcmt_servotubeCommand));
    __lcmt_servotubeCommand_clone_array(p, q, 1);
    return q;
}
 
void lcmt_servotubeCommand_destroy(lcmt_servotubeCommand *p)
{
    __lcmt_servotubeCommand_decode_array_cleanup(p, 1);
    free(p);
}
 
int lcmt_servotubeCommand_publish(lcm_t *lc, const char *channel, const lcmt_servotubeCommand *p)
{
      int max_data_size = lcmt_servotubeCommand_encoded_size (p);
      uint8_t *buf = (uint8_t*) malloc (max_data_size);
      if (!buf) return -1;
      int data_size = lcmt_servotubeCommand_encode (buf, 0, max_data_size, p);
      if (data_size < 0) {
          free (buf);
          return data_size;
      }
      int status = lcm_publish (lc, channel, buf, data_size);
      free (buf);
      return status;
}

struct _lcmt_servotubeCommand_subscription_t {
    lcmt_servotubeCommand_handler_t user_handler;
    void *userdata;
    lcm_subscription_t *lc_h;
};
static
void lcmt_servotubeCommand_handler_stub (const lcm_recv_buf_t *rbuf, 
                            const char *channel, void *userdata)
{
    int status;
    lcmt_servotubeCommand p;
    memset(&p, 0, sizeof(lcmt_servotubeCommand));
    status = lcmt_servotubeCommand_decode (rbuf->data, 0, rbuf->data_size, &p);
    if (status < 0) {
        fprintf (stderr, "error %d decoding lcmt_servotubeCommand!!!\n", status);
        return;
    }

    lcmt_servotubeCommand_subscription_t *h = (lcmt_servotubeCommand_subscription_t*) userdata;
    h->user_handler (rbuf, channel, &p, h->userdata);

    lcmt_servotubeCommand_decode_cleanup (&p);
}

lcmt_servotubeCommand_subscription_t* lcmt_servotubeCommand_subscribe (lcm_t *lcm, 
                    const char *channel, 
                    lcmt_servotubeCommand_handler_t f, void *userdata)
{
    lcmt_servotubeCommand_subscription_t *n = (lcmt_servotubeCommand_subscription_t*)
                       malloc(sizeof(lcmt_servotubeCommand_subscription_t));
    n->user_handler = f;
    n->userdata = userdata;
    n->lc_h = lcm_subscribe (lcm, channel, 
                                 lcmt_servotubeCommand_handler_stub, n);
    if (n->lc_h == NULL) {
        fprintf (stderr,"couldn't reg lcmt_servotubeCommand LCM handler!\n");
        free (n);
        return NULL;
    }
    return n;
}

int lcmt_servotubeCommand_unsubscribe(lcm_t *lcm, lcmt_servotubeCommand_subscription_t* hid)
{
    int status = lcm_unsubscribe (lcm, hid->lc_h);
    if (0 != status) {
        fprintf(stderr, 
           "couldn't unsubscribe lcmt_servotubeCommand_handler %p!\n", hid);
        return -1;
    }
    free (hid);
    return 0;
}

