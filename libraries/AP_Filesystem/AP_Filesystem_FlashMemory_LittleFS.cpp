/*
  ArduPilot filesystem interface for systems using the LittleFS filesystem in
  flash memory
*/

#include "AP_Filesystem.h"
#include "AP_Filesystem_FlashMemory_LittleFS.h"

#define HAL_FS_IN_FLASH_MEM_SIZE_BLOCKS 4  /* 256K for dataflash, 512K for W25N01GV */

#include "lfs.h"

#if 0
#define debug(fmt, args ...)  do {printf("%s:%d: " fmt "\n", __FUNCTION__, __LINE__, ## args); } while(0)
#else
#define debug(fmt, args ...)
#endif

#define ENSURE_MOUNTED() do { if (!mounted && !mount_filesystem()) { errno = EIO; return -1; }} while (0)
#define ENSURE_MOUNTED_NULL() do { if (!mounted && !mount_filesystem()) { errno = EIO; return nullptr; }} while (0)
#define LFS_CHECK(func) do { int __retval = func; if (__retval < 0) { errno = errno_from_lfs_error(__retval); return -1; }} while (0)
#define LFS_CHECK_NULL(func) do { int __retval = func; if (__retval < 0) { errno = errno_from_lfs_error(__retval); return nullptr; }} while (0)

static int flashmem_read(
    const struct lfs_config *cfg, lfs_block_t block, lfs_off_t off,
    void* buffer, lfs_size_t size
);
static int flashmem_prog(
    const struct lfs_config *cfg, lfs_block_t block, lfs_off_t off,
    const void* buffer, lfs_size_t size
);
static int flashmem_erase(const struct lfs_config *cfg, lfs_block_t block);
static int flashmem_sync(const struct lfs_config *cfg);

static int errno_from_lfs_error(int lfs_error);
static int lfs_flags_from_flags(int flags);

const extern AP_HAL::HAL& hal;

int AP_Filesystem_FlashMemory_LittleFS::open(const char *pathname, int flags, bool allow_absolute_path)
{
    int fd, retval;
    lfs_file_t* fp;

    WITH_SEMAPHORE(fs_sem);

    ENSURE_MOUNTED();

    fd = allocate_fd();
    if (fd < 0) {
        return -1;
    }

    fp = lfs_file_from_fd(fd);
    if (fp == nullptr) {
        return -1;
    }

    retval = lfs_file_open(&fs, fp, pathname, lfs_flags_from_flags(flags));
    if (retval < 0) {
        errno = errno_from_lfs_error(retval);
        free_fd(fd);
        return -1;
    }

    return fd;
}

int AP_Filesystem_FlashMemory_LittleFS::close(int fileno)
{
    lfs_file_t* fp;

    WITH_SEMAPHORE(fs_sem);

    fp = lfs_file_from_fd(fileno);
    if (fp == nullptr) {
        return -1;
    }

    LFS_CHECK(lfs_file_close(&fs, fp));

    if (free_fd(fileno) < 0) {
        return -1;
    }

    return 0;
}

int32_t AP_Filesystem_FlashMemory_LittleFS::read(int fd, void *buf, uint32_t count)
{
    lfs_file_t* fp;
    lfs_ssize_t read;

    WITH_SEMAPHORE(fs_sem);
    ENSURE_MOUNTED();

    fp = lfs_file_from_fd(fd);
    if (fp == nullptr) {
        return -1;
    }

    read = lfs_file_read(&fs, fp, buf, count);
    if (read < 0) {
        errno = errno_from_lfs_error(read);
        return -1;
    }

    return read;
}

int32_t AP_Filesystem_FlashMemory_LittleFS::write(int fd, const void *buf, uint32_t count)
{
    lfs_file_t* fp;
    lfs_ssize_t written;

    WITH_SEMAPHORE(fs_sem);
    ENSURE_MOUNTED();

    fp = lfs_file_from_fd(fd);
    if (fp == nullptr) {
        return -1;
    }
    
    written = lfs_file_write(&fs, fp, buf, count);
    if (written < 0) {
        errno = errno_from_lfs_error(written);
        return -1;
    }

    return written;
}

int AP_Filesystem_FlashMemory_LittleFS::fsync(int fd)
{
    lfs_file_t* fp;

    WITH_SEMAPHORE(fs_sem);
    ENSURE_MOUNTED();

    fp = lfs_file_from_fd(fd);
    if (fp == nullptr) {
        return -1;
    }

    LFS_CHECK(lfs_file_sync(&fs, fp));
    return 0;
}

int32_t AP_Filesystem_FlashMemory_LittleFS::lseek(int fd, int32_t position, int whence)
{
    lfs_file_t* fp;

    WITH_SEMAPHORE(fs_sem);
    ENSURE_MOUNTED();

    fp = lfs_file_from_fd(fd);
    if (fp == nullptr) {
        return -1;
    }

    LFS_CHECK(lfs_file_seek(&fs, fp, position, whence));
    return 0;
}

int AP_Filesystem_FlashMemory_LittleFS::stat(const char *name, struct stat *buf)
{
    lfs_info info;

    WITH_SEMAPHORE(fs_sem);
    ENSURE_MOUNTED();
    
    LFS_CHECK(lfs_stat(&fs, name, &info));

    memset(buf, 0, sizeof(*buf));
    buf->st_mode = (info.type == LFS_TYPE_DIR ? S_IFREG : S_IFDIR) | 0666;
    buf->st_nlink = 1;
    buf->st_size = info.size;
    buf->st_blksize = fs_cfg.read_size;    
    buf->st_blocks = (info.size >> 9) + ((info.size & 0x1FF) > 0 ? 1 : 0);

    return 0;
}

int AP_Filesystem_FlashMemory_LittleFS::unlink(const char *pathname)
{
    WITH_SEMAPHORE(fs_sem);
    ENSURE_MOUNTED();
    LFS_CHECK(lfs_remove(&fs, pathname));
    return 0;
}

int AP_Filesystem_FlashMemory_LittleFS::mkdir(const char *pathname)
{
    WITH_SEMAPHORE(fs_sem);
    ENSURE_MOUNTED();
    LFS_CHECK(lfs_mkdir(&fs, pathname));
    return 0;
}

typedef struct {
    lfs_dir_t dir;
    struct dirent entry;
} lfs_dir_entry_pair;

void *AP_Filesystem_FlashMemory_LittleFS::opendir(const char *pathdir)
{
    int retval;

    WITH_SEMAPHORE(fs_sem);
    ENSURE_MOUNTED_NULL();

    lfs_dir_entry_pair *result = new lfs_dir_entry_pair;
    if (!result) {
        errno = ENOMEM;
        return nullptr;
    }

    retval = lfs_dir_open(&fs, &result->dir, pathdir);
    if (retval < 0) {
        delete result;
        errno = errno_from_lfs_error(retval);
        return nullptr;
    }

    memset(&result->entry, 0, sizeof(result->entry));
    result->entry.d_reclen = sizeof(result->entry);

    return result;
}

struct dirent *AP_Filesystem_FlashMemory_LittleFS::readdir(void *ptr)
{
    WITH_SEMAPHORE(fs_sem);

    lfs_info info;
    lfs_dir_entry_pair *pair = static_cast<lfs_dir_entry_pair*>(ptr);
    if (!pair) {
        errno = EINVAL;
        return nullptr;
    }

    if (!lfs_dir_read(&fs, &pair->dir, &info)) {
        /* no more entries */
        return nullptr;
    }

    memset(&pair->entry, 0, sizeof(pair->entry));

    pair->entry.d_ino = 0;
    pair->entry.d_seekoff++;

    strncpy(pair->entry.d_name, info.name, sizeof(pair->entry.d_name));
    pair->entry.d_namlen = strlen(info.name);

    pair->entry.d_type = info.type == LFS_TYPE_DIR ? DT_DIR : DT_REG;

    return &pair->entry;
}

int AP_Filesystem_FlashMemory_LittleFS::closedir(void *ptr)
{
    WITH_SEMAPHORE(fs_sem);

    lfs_dir_entry_pair *pair = static_cast<lfs_dir_entry_pair*>(ptr);
    if (!pair) {
        errno = EINVAL;
        return 0;
    }

    LFS_CHECK(lfs_dir_close(&fs, &pair->dir));

    delete pair;

    return 0;
}

int64_t AP_Filesystem_FlashMemory_LittleFS::disk_free(const char *path)
{
    lfs_ssize_t alloc_size;

    WITH_SEMAPHORE(fs_sem);
    ENSURE_MOUNTED();

    alloc_size = lfs_fs_size(&fs);
    if (alloc_size < 0) {
        errno = errno_from_lfs_error(alloc_size);
        return -1;
    }

    return disk_space(path) - alloc_size;
}

int64_t AP_Filesystem_FlashMemory_LittleFS::disk_space(const char *path)
{
    return fs_cfg.block_count * fs_cfg.block_size;
}

bool AP_Filesystem_FlashMemory_LittleFS::retry_mount(void)
{
    WITH_SEMAPHORE(fs_sem);

    if (!dead) {
        if (!mounted && !mount_filesystem()) {
            errno = EIO;
            return false;
        }

        return true;
    } else {
        return false;
    }
}

void AP_Filesystem_FlashMemory_LittleFS::unmount(void)
{
    WITH_SEMAPHORE(fs_sem);

    if (mounted && !dead) {
        if (lfs_unmount(&fs) >= 0) {
            mounted = false;
        }
    }
}

/* ************************************************************************* */
/* Private functions                                                         */
/* ************************************************************************* */

int AP_Filesystem_FlashMemory_LittleFS::allocate_fd()
{
    int fd;

    for (fd = 0; fd < MAX_OPEN_FILES; fd++) {
        if (open_files[fd] == nullptr) {
            open_files[fd] = static_cast<lfs_file_t*>(calloc(1, sizeof(lfs_file_t)));
            if (open_files[fd] == nullptr) {
                errno = ENOMEM;
                return -1;
            }

            return fd;
        }
    }

    errno = ENFILE;
    return -1;
}

int AP_Filesystem_FlashMemory_LittleFS::free_fd(int fd)
{
    lfs_file_t* fp = lfs_file_from_fd(fd);
    if (!fp) {
        return -1;
    }

    free(fp);
    open_files[fd] = fp = nullptr;

    return 0;
}

void AP_Filesystem_FlashMemory_LittleFS::free_all_fds()
{
    int fd;

    for (fd = 0; fd < MAX_OPEN_FILES; fd++) {
        if (open_files[fd] == nullptr) {
            free_fd(fd);
        }
    }
}

lfs_file_t* AP_Filesystem_FlashMemory_LittleFS::lfs_file_from_fd(int fd) const
{
    if (fd < 0 || fd >= MAX_OPEN_FILES || open_files[fd] == nullptr) {
        errno = EBADF;
        return nullptr;
    }

    return open_files[fd];
}

void AP_Filesystem_FlashMemory_LittleFS::mark_dead()
{
    if (!dead) {
        printf("FlashMemory_LittleFS: dead\n");
        free_all_fds();
        dead = true;
    }
}

/* ************************************************************************* */
/* Low-level flash memory access                                             */
/* ************************************************************************* */

#define JEDEC_WRITE_ENABLE           0x06
#define JEDEC_READ_STATUS            0x05
#define JEDEC_READ_DATA              0x03
#define JEDEC_DEVICE_ID              0x9F
#define JEDEC_PAGE_WRITE             0x02

#define JEDEC_BULK_ERASE             0xC7
#define JEDEC_SECTOR4_ERASE          0x20 // 4k erase
#define JEDEC_BLOCK32_ERASE          0x52 // 32K erase
#define JEDEC_BLOCK64_ERASE          0xD8 // 64K erase

#define JEDEC_STATUS_BUSY            0x01
#define JEDEC_STATUS_WRITEPROTECT    0x02
#define JEDEC_STATUS_BP0             0x04
#define JEDEC_STATUS_BP1             0x08
#define JEDEC_STATUS_BP2             0x10
#define JEDEC_STATUS_TP              0x20
#define JEDEC_STATUS_SEC             0x40
#define JEDEC_STATUS_SRP0            0x80

/*
  flash device IDs taken from betaflight flash_m25p16.c

  Format is manufacturer, memory type, then capacity
*/
#define JEDEC_ID_MACRONIX_MX25L3206E   0xC22016
#define JEDEC_ID_MACRONIX_MX25L6406E   0xC22017
#define JEDEC_ID_MACRONIX_MX25L25635E  0xC22019
#define JEDEC_ID_MICRON_M25P16         0x202015
#define JEDEC_ID_MICRON_N25Q064        0x20BA17
#define JEDEC_ID_MICRON_N25Q128        0x20ba18
#define JEDEC_ID_WINBOND_W25Q16        0xEF4015
#define JEDEC_ID_WINBOND_W25Q32        0xEF4016
#define JEDEC_ID_WINBOND_W25X32        0xEF3016
#define JEDEC_ID_WINBOND_W25Q64        0xEF4017
#define JEDEC_ID_WINBOND_W25Q128       0xEF4018
#define JEDEC_ID_WINBOND_W25Q256       0xEF4019
#define JEDEC_ID_WINBOND_W25Q128_2     0xEF7018
#define JEDEC_ID_CYPRESS_S25FL128L     0x016018

uint8_t AP_Filesystem_FlashMemory_LittleFS::read_status_register()
{
    WITH_SEMAPHORE(dev_sem);
    uint8_t cmd = JEDEC_READ_STATUS;
    uint8_t status;
    dev->transfer(&cmd, 1, &status, 1);
    return status;
}

bool AP_Filesystem_FlashMemory_LittleFS::is_busy()
{
    return (read_status_register() & (JEDEC_STATUS_BUSY | JEDEC_STATUS_SRP0)) != 0;
}

void AP_Filesystem_FlashMemory_LittleFS::send_command_addr(uint8_t command, uint32_t addr)
{
    uint8_t cmd[5];
    cmd[0] = command;

    if (use_32bit_address) {
        cmd[1] = (addr >> 24) & 0xff;
        cmd[2] = (addr >> 16) & 0xff;
        cmd[3] = (addr >>  8) & 0xff;
        cmd[4] = (addr >>  0) & 0xff;
    } else {
        cmd[1] = (addr >> 16) & 0xff;
        cmd[2] = (addr >>  8) & 0xff;
        cmd[3] = (addr >>  0) & 0xff;
    }

    dev->transfer(cmd, use_32bit_address ? 5 : 4, nullptr, 0);
}

void AP_Filesystem_FlashMemory_LittleFS::wait_until_device_is_ready()
{
    if (dead) {
        return;
    }

    uint32_t t = AP_HAL::millis();
    while (is_busy()) {
        hal.scheduler->delay_microseconds(100);
        if (AP_HAL::millis() - t > 5000) {
            mark_dead();
            break;
        }
    }
}

bool AP_Filesystem_FlashMemory_LittleFS::find_block_size_and_count() {
    wait_until_device_is_ready();

    WITH_SEMAPHORE(dev_sem);

    // Read manufacturer ID
    uint8_t cmd = JEDEC_DEVICE_ID;
    uint8_t buf[4];
    dev->transfer(&cmd, 1, buf, 4);

    uint32_t id = buf[0] << 16 | buf[1] << 8 | buf[2];

    uint32_t blocks = 0;
    uint16_t pages_per_block = 0;
    uint16_t pages_per_sector = 0;
    uint16_t page_size = 0;

    use_32bit_address = false;

    switch (id) {
    case JEDEC_ID_WINBOND_W25Q16:
    case JEDEC_ID_MICRON_M25P16:
        blocks = 32;
        pages_per_block = 256;
        pages_per_sector = 16;
        page_size = 256;
        break;

    case JEDEC_ID_WINBOND_W25Q32:
    case JEDEC_ID_WINBOND_W25X32:
    case JEDEC_ID_MACRONIX_MX25L3206E:
        blocks = 64;
        pages_per_block = 256;
        pages_per_sector = 16;
        page_size = 256;
        break;

    case JEDEC_ID_MICRON_N25Q064:
    case JEDEC_ID_WINBOND_W25Q64:
    case JEDEC_ID_MACRONIX_MX25L6406E:
        blocks = 128;
        pages_per_block = 256;
        pages_per_sector = 16;
        page_size = 256;
        break;

    case JEDEC_ID_MICRON_N25Q128:
    case JEDEC_ID_WINBOND_W25Q128:
    case JEDEC_ID_WINBOND_W25Q128_2:
    case JEDEC_ID_CYPRESS_S25FL128L:
        blocks = 256;
        pages_per_block = 256;
        pages_per_sector = 16;
        page_size = 256;
        break;

    case JEDEC_ID_WINBOND_W25Q256:
    case JEDEC_ID_MACRONIX_MX25L25635E:
        blocks = 512;
        pages_per_block = 256;
        pages_per_sector = 16;
        page_size = 256;
        use_32bit_address = true;
        break;

    default:
        hal.scheduler->delay(2000);
        printf("Unknown SPI Flash 0x%08x\n", id);
        return false;
    }

    if (blocks < HAL_FS_IN_FLASH_MEM_SIZE_BLOCKS) {
        printf(
            "SPI flash too small for LittleFS, needs %d blocks, got %d\n",
            HAL_FS_IN_FLASH_MEM_SIZE_BLOCKS, blocks
        );
        return false;
    }

    fs_cfg.read_size = page_size;
    fs_cfg.prog_size = page_size;
    fs_cfg.block_size = pages_per_sector * page_size;
    fs_cfg.block_count = (pages_per_block / pages_per_sector) * HAL_FS_IN_FLASH_MEM_SIZE_BLOCKS;

    address_offset = (
        blocks * (pages_per_block / pages_per_sector)
        - fs_cfg.block_count
    ) * fs_cfg.block_size;

    return true;
}

bool AP_Filesystem_FlashMemory_LittleFS::mount_filesystem() {
    if (dead) {
        return false;
    }

    if (mounted) {
        return true;
    }

    fs_cfg.context = this;

    fs_cfg.read = flashmem_read;
    fs_cfg.prog = flashmem_prog;
    fs_cfg.erase = flashmem_erase;
    fs_cfg.sync = flashmem_sync;

    fs_cfg.block_cycles = 500;
    fs_cfg.cache_size = 4096;
    fs_cfg.lookahead_size = 16;

    dev = hal.spi->get_device("dataflash");
    if (!dev) {
        mark_dead();
        return false;
    }

    dev_sem = dev->get_semaphore();

    if (!find_block_size_and_count()) {
        mark_dead();
        return false;
    }

    // find_block_size_and_count() filled out the read / prog / block sizes
    // and counts in fs_cfg so it's time to mount

    if (lfs_mount(&fs, &fs_cfg) < 0) {
        /* maybe not formatted? try formatting it */
        printf("FlashMemory_LittleFS: formatting filesystem\n");
        if (lfs_format(&fs, &fs_cfg) < 0) {
            /* cannot format either, give up */
            mark_dead();
            return false;
        }

        /* try mounting again */
        if (lfs_mount(&fs, &fs_cfg) < 0) {
            /* cannot mount after formatting */
            mark_dead();
            return false;
        }
    }

    mounted = true;
    return true;
}

uint32_t AP_Filesystem_FlashMemory_LittleFS::lfs_block_to_raw_flash_address(lfs_block_t index)
{
    return address_offset + index * fs_cfg.block_size;
}

void AP_Filesystem_FlashMemory_LittleFS::flashmem_enable_write()
{
    uint8_t b = JEDEC_WRITE_ENABLE;

    wait_until_device_is_ready();

    {
        WITH_SEMAPHORE(dev_sem);
        dev->transfer(&b, 1, nullptr, 0);
    }
}

int AP_Filesystem_FlashMemory_LittleFS::_flashmem_read(
    lfs_block_t block, lfs_off_t off, void* buffer, lfs_size_t size
) {
    wait_until_device_is_ready();

    WITH_SEMAPHORE(dev_sem);

    dev->set_chip_select(true);
    send_command_addr(JEDEC_READ_DATA, lfs_block_to_raw_flash_address(block) + off);
    dev->transfer(nullptr, 0, static_cast<uint8_t*>(buffer), size);
    dev->set_chip_select(false);

    return 0;
}

int AP_Filesystem_FlashMemory_LittleFS::_flashmem_prog(
    lfs_block_t block, lfs_off_t off, const void* buffer, lfs_size_t size
) {
    flashmem_enable_write();
    wait_until_device_is_ready();

    {
        WITH_SEMAPHORE(dev_sem);

        dev->set_chip_select(true);
        send_command_addr(JEDEC_PAGE_WRITE, lfs_block_to_raw_flash_address(block) + off);
        dev->transfer(static_cast<const uint8_t*>(buffer), size, nullptr, 0);
        dev->set_chip_select(false);
    }

    return 0;
}

int AP_Filesystem_FlashMemory_LittleFS::_flashmem_erase(lfs_block_t block) {
    flashmem_enable_write();
    wait_until_device_is_ready();

    {
        WITH_SEMAPHORE(dev_sem);
        send_command_addr(JEDEC_SECTOR4_ERASE, lfs_block_to_raw_flash_address(block));
    }

    return 0;
}

int AP_Filesystem_FlashMemory_LittleFS::_flashmem_sync() {
    /* Nothing to do, no write caching */
    return 0;
}

static int flashmem_read(
    const struct lfs_config *cfg, lfs_block_t block, lfs_off_t off,
    void* buffer, lfs_size_t size
) {
    AP_Filesystem_FlashMemory_LittleFS* self = static_cast<AP_Filesystem_FlashMemory_LittleFS*>(cfg->context);
    return self->_flashmem_read(block, off, buffer, size);
}

static int flashmem_prog(
    const struct lfs_config *cfg, lfs_block_t block, lfs_off_t off,
    const void* buffer, lfs_size_t size
) {
    AP_Filesystem_FlashMemory_LittleFS* self = static_cast<AP_Filesystem_FlashMemory_LittleFS*>(cfg->context);
    return self->_flashmem_prog(block, off, buffer, size);
}

static int flashmem_erase(const struct lfs_config *cfg, lfs_block_t block) {
    AP_Filesystem_FlashMemory_LittleFS* self = static_cast<AP_Filesystem_FlashMemory_LittleFS*>(cfg->context);
    return self->_flashmem_erase(block);
}

static int flashmem_sync(const struct lfs_config *cfg) {
    AP_Filesystem_FlashMemory_LittleFS* self = static_cast<AP_Filesystem_FlashMemory_LittleFS*>(cfg->context);
    return self->_flashmem_sync();
}

/* ************************************************************************* */
/* LittleFS to POSIX API conversion functions                                */
/* ************************************************************************* */

static int errno_from_lfs_error(int lfs_error)
{
    switch (lfs_error) {
        case LFS_ERR_OK: return 0;
        case LFS_ERR_IO: return EIO;
        case LFS_ERR_CORRUPT: return EIO;
        case LFS_ERR_NOENT: return ENOENT;
        case LFS_ERR_EXIST: return EEXIST;
        case LFS_ERR_NOTDIR: return ENOTDIR;
        case LFS_ERR_ISDIR: return EISDIR;
        case LFS_ERR_NOTEMPTY: return ENOTEMPTY;
        case LFS_ERR_BADF: return EBADF;
        case LFS_ERR_FBIG: return EFBIG;
        case LFS_ERR_INVAL: return EINVAL;
        case LFS_ERR_NOSPC: return ENOSPC;
        case LFS_ERR_NOMEM: return ENOMEM;
        case LFS_ERR_NOATTR: return ENOATTR;
        case LFS_ERR_NAMETOOLONG: return ENAMETOOLONG;
        default: return EIO;
    }
}

static int lfs_flags_from_flags(int flags)
{
    int outflags = 0;

    if (flags & O_WRONLY) {
        outflags |= LFS_O_WRONLY;
    } else if (flags & O_RDWR) {
        outflags |= LFS_O_RDWR;
    } else {
        outflags |= LFS_O_RDONLY;
    }

    if (flags & O_CREAT) {
        outflags |= LFS_O_CREAT;
    }

    if (flags & O_EXCL) {
        outflags |= LFS_O_EXCL;
    }
    
    if (flags & O_TRUNC) {
        outflags |= LFS_O_TRUNC;
    }
    
    if (flags & O_APPEND) {
        outflags |= LFS_O_APPEND;
    }
    
    return outflags;
}
