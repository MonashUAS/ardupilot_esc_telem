#pragma once

#include "DataFlash_Backend.h"

#include <AP_Mission/AP_Mission.h>

class DFMessageWriter {
public:

    virtual void reset() = 0;
    virtual void process() = 0;
    virtual bool finished() const { return _finished; };

    virtual void set_dataflash_backend(class DataFlash_Backend *backend) {
        _dataflash_backend = backend;
    }

    virtual bool fmt_done() const;
    virtual void set_mission(const AP_Mission *mission) { }

protected:
    bool _finished = false;
    DataFlash_Backend *_dataflash_backend = nullptr;

private:
};


class DFMessageWriter_WriteFormats : public DFMessageWriter {
public:
    DFMessageWriter_WriteFormats() :
        DFMessageWriter()
        { }

    void reset();
    void process();

    bool fmt_done() const override { return stage == stage_done; };

private:
    enum stage_t {
        stage_init,
        stage_formats,
        stage_done
    };
    stage_t stage = stage_init;

    uint16_t next_format_to_send;

};


class DFMessageWriter_WriteSysInfo : public DFMessageWriter {
public:
    DFMessageWriter_WriteSysInfo(const char *firmware_string) :
        DFMessageWriter(),
        _firmware_string(firmware_string)
        { }

    void reset();
    void process();

private:
    enum write_sysinfo_blockwriter_stage {
        ws_blockwriter_stage_init,
        ws_blockwriter_stage_firmware_string,
        ws_blockwriter_stage_git_versions,
        ws_blockwriter_stage_system_id
    };
    write_sysinfo_blockwriter_stage stage = ws_blockwriter_stage_init;

    const char *_firmware_string;
};

class DFMessageWriter_WriteEntireMission : public DFMessageWriter {
public:

    void reset();
    void process();

    void set_mission(const AP_Mission *mission);

private:
    enum entire_mission_blockwriter_stage {
        em_blockwriter_stage_init,
        em_blockwriter_stage_write_new_mission_message,
        em_blockwriter_stage_write_mission_items,
        em_blockwriter_stage_done
    };

    const AP_Mission *_mission = nullptr;
    uint16_t _mission_number_to_send = 0;
    entire_mission_blockwriter_stage stage = em_blockwriter_stage_init;
};

class DFMessageWriter_DFLogStart : public DFMessageWriter {
public:
    DFMessageWriter_DFLogStart(const char *firmware_string) :
        _writesysinfo(firmware_string),
        _writeentiremission()
        {
        }

    virtual void set_dataflash_backend(class DataFlash_Backend *backend) {
        DFMessageWriter::set_dataflash_backend(backend);
        _writeformats.set_dataflash_backend(backend);
        _writesysinfo.set_dataflash_backend(backend);
        _writeentiremission.set_dataflash_backend(backend);
    }

    void reset();
    void process();

    void set_mission(const AP_Mission *mission);
    bool fmt_done() const override { return _writeformats.finished(); };

private:

    enum log_start_blockwriter_stage {
        ls_blockwriter_stage_init,
        ls_blockwriter_stage_formats,
        ls_blockwriter_stage_parms,
        ls_blockwriter_stage_sysinfo,
        ls_blockwriter_stage_write_entire_mission,
        ls_blockwriter_stage_vehicle_messages,
        ls_blockwriter_stage_done,
    };

    log_start_blockwriter_stage stage = ls_blockwriter_stage_init;

    AP_Param::ParamToken token;
    AP_Param *ap;
    enum ap_var_type type;


    DFMessageWriter_WriteFormats _writeformats;
    DFMessageWriter_WriteSysInfo _writesysinfo;
    DFMessageWriter_WriteEntireMission _writeentiremission;
};
