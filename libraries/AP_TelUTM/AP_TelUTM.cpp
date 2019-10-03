
#include <stdint.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <wolfmqtt/mqtt_client.h>

extern const AP_HAL::HAL& hal;

static AP_TelUTM _security;

AP_TelUTM::AP_TelUTM()
{
    if (_singleton != nullptr) {
        AP_HAL::panic("Too many TelUTM modules");
        return;
    }
    _singleton = this;
}

static int mqttclient_tls_verify_cb(int preverify, WOLFSSL_X509_STORE_CTX* store)
{
  char buffer[WOLFSSL_MAX_ERROR_SZ];

  printf("MQTT TLS Verify Callback: PreVerify %d, Error %d (%s)\n", preverify,
         store->error, wolfSSL_ERR_error_string(store->error, buffer));
  printf("  Subject's domain name is %s\n", store->domain);

  if (store->error != 0) {
    /* Allowing to continue */
    /* Should check certificate and return 0 if not okay */
    printf("  Allowing cert anyways");
  }

  return 1;
}

static int mqttclient_tls_cb(MqttClient* cli)
{
  int rc = WOLFSSL_FAILURE;
  
  cli->tls.ctx = wolfSSL_CTX_new(wolfTLSv1_2_client_method());
  if (cli->tls.ctx) {
    wolfSSL_CTX_set_verify(cli->tls.ctx, SSL_VERIFY_PEER, mqttclient_tls_verify_cb);

    /* default to success */
    rc = WOLFSSL_SUCCESS;

    if (mTlsFile) {
      /* Load CA certificate file */
      rc = wolfSSL_CTX_load_verify_locations(cli->tls.ctx, mTlsFile, 0);
    }
  }
  else {
#if 0
    /* Load CA using buffer */
    rc = wolfSSL_CTX_load_verify_buffer(cli->tls.ctx, caCertBuf,
                                        caCertSize, WOLFSSL_FILETYPE_PEM);
#endif
    rc = WOLFSSL_SUCCESS;
  }

  printf("MQTT TLS Setup (%d)\n", rc);

  return rc;
}


// singleton instance
AP_TelUTM *AP_TelUTM::_singleton;

namespace AP {

AP_TelUTM &TelUTM()
{
    return *AP_TelUTM::get_singleton();
}

}