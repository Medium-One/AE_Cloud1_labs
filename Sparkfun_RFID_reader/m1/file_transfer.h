/*
 * file_transfer.h
 *
 *  Created on: Apr 21, 2017
 *      Author: demo
 */

#ifndef FILE_TRANSFER_H_
#define FILE_TRANSFER_H_


#define FILENAME_LENGTH 128


typedef struct {
    const hash_instance_t * p_hash;
    const crypto_instance_t * p_crypto;
} crypto_config_t;

typedef struct {
    FX_MEDIA * p_media;
    char filename[FILENAME_LENGTH];
    char * save_path;
} file_config_t;

typedef struct {
    NX_IP * p_ip;
    NX_PACKET_POOL * p_ppool;
    NX_DNS * p_dns;
    NX_SECURE_TLS_SESSION * p_tls_session;
    NX_SECURE_X509_CERT * p_trusted_cert;
} net_config_t;

typedef struct {
    void (* processing_callback) (int disk, int network);
    net_config_t net_config;
    file_config_t file_config;
    crypto_config_t crypto_config;
} ota_config_t;


int update_firmware(char * update_url,
                    uint32_t * update_hash,
                    ota_config_t * ota_config);


#endif /* FILE_TRANSFER_H_ */
