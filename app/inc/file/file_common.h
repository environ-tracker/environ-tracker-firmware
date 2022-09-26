#include <fs/fs.h>


#define FILE_NAME_LEN CONFIG_FILE_SYSTEM_MAX_FILE_NAME

extern struct fs_mount_t *mp;

/**
 * @brief Creates the absolute file name from a relative one.
 * 
 * NOTE: Path must be **at least** FILE_NAME_LEN in size.
 * 
 * @param path Destination to write absolute file name to
 * @param fname Relative file name
 * @return 0 on success, or number of bytes that would've been written
 */
int absolute_file_name(char *path, char *fname);

/**
 * @brief Reads from the file given by fname.
 * 
 * NOTE: This function handles both the opening and closing of the file.
 * 
 * @param fname Name of file to open
 * @param data Location to read data to
 * @param len Length of data to read
 * @return 0 on success, or negative error code from FS API
 */
int read_file(char *fname, uint8_t *data, uint32_t len);

/**
 * @brief Writes to the file given by fname. Creates the file if it doesn't 
 *        exist.
 * 
 * NOTE: This function handles both the opening and closing of the file.
 * 
 * @param fname Name of file to open
 * @param data Data to write to the file
 * @param len Length of the data to write
 * @return 0 on success, or negative error code from FS API
 */
int write_file(char *fname, uint8_t *data, uint32_t len);

/**
 * @brief Searches for the given file in the directory.
 * 
 * NOTE: dir_name is an absolute path, file_name is a relative path.
 * 
 * @param dir_name Directory to search
 * @param file_name File to search for
 * @return 0 on success, else negative error code
 */
int search_directory(char *dir_name, char *file_name);