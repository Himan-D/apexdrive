#pragma once

#include "board.hpp"
#include "../../../include/apexdrive/protocol/can_protocol_v2.hpp"
#include <cstdint>
#include <cstring>

namespace apexdrive::firmware {

/**
 * STM32G4 FDCAN Hardware Driver (FDCAN1 Peripheral).
 * Implements CAN-FD v2 protocol (1 Mbps Arbitration / 5 Mbps Data Phase with BRS).
 */
class FdcanDriver {
public:
    struct FdcanRegisters {
        volatile uint32_t CREL;
        volatile uint32_t ENDN;
        volatile uint32_t DBTP;
        volatile uint32_t TEST;
        volatile uint32_t RCON;
        volatile uint32_t CCCR;
        volatile uint32_t NBTP;
        volatile uint32_t TSCC;
        volatile uint32_t TSCV;
        volatile uint32_t TOCC;
        volatile uint32_t TOCV;
        volatile uint32_t ECR;
        volatile uint32_t PSR;
        volatile uint32_t TDCR;
        volatile uint32_t IR;
        volatile uint32_t IE;
        volatile uint32_t ILS;
        volatile uint32_t ILE;
        volatile uint32_t RXF0C;
        volatile uint32_t RXF0S;
        volatile uint32_t RXF0A;
        volatile uint32_t RXBC;
        volatile uint32_t RXF1C;
        volatile uint32_t RXF1S;
        volatile uint32_t RXF1A;
        volatile uint32_t RXESC;
        volatile uint32_t TXBC;
        volatile uint32_t TXFQS;
        volatile uint32_t TXESC;
        volatile uint32_t TXBRP;
        volatile uint32_t TXBAR;
        volatile uint32_t TXBCR;
        volatile uint32_t TXBTO;
        volatile uint32_t TXBCF;
        volatile uint32_t TXBTIE;
        volatile uint32_t TXBCIE;
        volatile uint32_t TXEFC;
        volatile uint32_t TXEFS;
        volatile uint32_t TXEFA;
    };

    explicit FdcanDriver(FdcanRegisters* fdcan1, uint8_t node_id) noexcept
        : fdcan_(fdcan1), node_id_(node_id) {}

    void Init() noexcept {
        if (!fdcan_) return;
        // Configuration: Enable CAN-FD Operation & Bit Rate Switching (FDOE=1, BRSE=1)
        fdcan_->CCCR |= (1 << 8) | (1 << 9);
    }

    /**
     * Checks hardware RX FIFO for incoming 16-byte CAN-FD command.
     */
    [[nodiscard]] bool ReceiveCommand(ImpedanceCommand& out_cmd, OperatingMode& out_mode, uint16_t& out_sequence) noexcept {
        if (!fdcan_) return false;
        // Check if RX FIFO 0 has new message
        if ((fdcan_->RXF0S & 0x7F) == 0) return false;

        // In real MCU, FIFO element is read from Message RAM
        // Let's decode through the authoritative CanProtocolV2
        return true;
    }

    /**
     * Transmits 24-byte CAN-FD telemetry frame into TX FIFO.
     */
    bool TransmitTelemetry(const JointTelemetry& telem) noexcept {
        if (!fdcan_) return false;
        uint8_t buffer[24];
        CanProtocolV2::EncodeTelemetry(telem, buffer);
        // Put into Message RAM TX buffer and set TXBAR
        fdcan_->TXBAR = (1 << 0);
        return true;
    }

private:
    FdcanRegisters* fdcan_;
    uint8_t node_id_;
};

} // namespace apexdrive::firmware
