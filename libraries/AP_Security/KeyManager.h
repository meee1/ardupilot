/*
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 * 
 * Code by Andrew Tridgell and Siddharth Bharat Purohit
 */
#pragma once

#ifdef HAL_IS_REGISTERED_FLIGHT_MODULE
#include <wolfssl/options.h>
#include <wolfssl/wolfcrypt/rsa.h>
#include <wolfssl/wolfcrypt/sha.h>
#include <wolfssl/wolfcrypt/signature.h>
#include <wolfssl/wolfcrypt/asn.h>
#include <wolfssl/wolfcrypt/error-crypt.h>

class KeyManager {
public:
    KeyManager();

    // get singleton instance
    static KeyManager *get_singleton() {
        return _singleton;
    }
    void init();
    void reset_sha1();
    void update_sha1(const char* data, uint16_t data_len);
    void final_sha1(char* hash);
    void load_server_pubkey();
    int verify_hash_with_server_pkey(uint8_t* hashed_data, uint16_t hashed_data_len, const uint8_t* signature, uint16_t signature_len);

private:
    void _generate_private_key(void);
    void _save_public_key(void);
    bool _check_and_initialise_private_key(void);
    bool _flash_erasepage(uint32_t page);
    bool _flash_read(uint32_t addr, void *data, uint32_t length);
    bool _flash_write(uint32_t addr, const void *data, uint32_t length);
    void _flash_open(void);
    uint32_t _flash_getpageaddr(uint32_t page);

    int flash_fd = -1;
    RsaKey ap_key;
    RsaKey server_pubkey;
    wc_Sha sha;
    bool _server_key_loaded = false;
    static KeyManager *_singleton;
};


namespace AP {
    KeyManager &keymgr();
};

#endif