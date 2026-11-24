/**
 * @file index_html.h
 * @brief Accessor interface for embedded HTTP dashboard HTML asset.
 */

#ifndef INDEX_HTML_H
#define INDEX_HTML_H

#include <stdint.h>

#define INDEX_HTML_BODY_LEN             (44339u)

/**
 * @brief Return pointer to the NUL-terminated embedded dashboard HTML document.
 * @return Read-only pointer to static HTML string in flash.
 */
const char * index_html_get_body (void);

/**
 * @brief Return byte length of the embedded dashboard HTML document.
 * @return Byte count excluding trailing NUL terminator.
 */
uint32_t index_html_get_length (void);

#endif /* INDEX_HTML_H */

/*** end of file ***/
