#!/usr/bin/env ruby
# terminal.rb: A simple interactive terminal for accesssing the EEPROM burner
# (see `burner.c`).
#
# Passes most commands through to the burner as they are, but supports two
# others:
#
#    * `read file.hex` or `read file.hex start end` – Read the EEPROM
#      (or optionally the address range from `start` to `end`) to the
#      `file.hex` in Intel HEX format.
#    * `burn file.hex` – Burn `file.hex` (which must be in Intel HEX
#      format) to the EEPROM.
#
# Copyright (c) 2019 Kimmo Kulovesi, https://arkku.com/
# Provided with absolutely no warranty, use at your own risk only.
# Use and distribute freely, mark modified copies as such.
##############################################################################

require 'rubygems'
require 'serialport'

class Burner
  attr_reader :serial
  attr_reader :xon

  def initialize(serial_device: '/dev/ttyUSB0', baud: 57600)
    @serial = SerialPort.new(serial_device, baud, 8, 1, SerialPort::NONE)
    @xon = true
    @serial.autoclose = true
    @serial.flow_control = SerialPort::NONE # We will handle XON/XOFF manually
    @ihex_destination = nil
    @ihex_source = nil
    @mutex = Mutex.new
  end

  def read
    result = ''
    loop do
      r = @serial.read(1)
      break if r.nil?
      case r
      when "\n"
        break
      when "\r"
        next
      when "\u0011"
        xon
        break if result.empty?
      when "\u0013"
        xoff
        break if result.empty?
      when "\t"
        result << ' '
      when /[\u0000-\u0019]/
        next
      else
        result << r
      end
    end
    @mutex.synchronize do
      if result =~ /Error:/ || result =~ /^! (Write|Read) end/
        if @ihex_source
          if @previous_ihex && (result =~ /checksum/ || result =~ /IHEX/)
            # Error in the IHEX data
            log "! Resend: #{result}"
            @serial.write("\r\n")
            send(@previous_ihex)
            xoff # Prevent another new line from being sent until next XON
          else
            @ihex_source.close
            @ihex_source = nil
            log "! Write terminated: #{result}"
            send('!') # Break out of IHEX mode
          end
        end
        if @ihex_destination
          @ihex_destination.close
          @ihex_destination = nil
          log "! Read terminated: #{result}"
        end
      end
      if @ihex_destination && (ihex_data = result.slice!(/:[0-9A-Fa-f]+/))
        @ihex_destination.puts(ihex_data)
        if ihex_data =~ /^:00....01..$/
          @ihex_destination.close
          @ihex_destination = nil
          log "! Read completed"
        elsif ihex_data =~ /^:....0000/
          log ">> Reading: #{ihex_data}"
        end
      end
      if @ihex_source && @xon
        if (line = @ihex_source.gets) && line =~ /^:/
          @previous_ihex = line
          send(line, logging: false)
        else
          if line.nil?
            log "! End of input file"
          else
            log "! Invalid input: #{line.inspect}"
            send(':00000001FF', logging: false)
            send('!')
          end
          @ihex_source.close
          @ihex_source = nil
        end
      end
    end
    result
  end

  def xon
    @mutex.synchronize do
      unless @xon
        #log "! XON"
        @xon = true
      end
    end
  end

  def xoff
    @mutex.synchronize do
      if @xon
        #log "! xoff"
        @xon = false
      end
    end
  end

  def read_to_file(filename, first: nil, last: nil)
    @mutex.synchronize do
      @ihex_destination.close if @ihex_destination
      file = File.new(filename, 'wt')
      @ihex_destination = file
      if file.nil?
        log "! Error: #{filename}"
        return
      end
      if last
        send("R $#{'%04X' % (first || 0)} $#{'%04X' % last}")
      elsif first
        send("R $#{'%04X' % (first)}")
      else
        send('R')
      end
      log "! Reading to IHEX file: #{filename}"
    end
  end

  def burn_from_ihex_file(filename, first: nil, last: nil)
    @mutex.synchronize do
      @ihex_source.close if @ihex_source
      @previous_ihex = nil
      begin
        file = File.open(filename, 'rt')
        @ihex_source = file
      rescue Exception => e
        log "! Error: #{filename}: #{e}"
        return
      end
      send('W')
      log "! Burning from IHEX file: #{filename}"
    end
  end

  def send(msg, logging: true)
    msg = "#{msg.to_s}\r\n"
    log "<< #{msg.inspect}" if logging
    @serial.write(msg)
  end

  def send_sync(msg)
    @mutex.synchronize do
      send(msg)
    end
  end

  def log(msg)
    $stderr.puts(msg)
  end
end

port = ARGV.first.to_s
port = '/dev/ttyUSB0' if port.empty?
baud = ARGV.last.to_i
baud = 57600 if baud <= 0

burner = Burner.new(serial_device: port, baud: baud)

$keep_running = true

open("/dev/tty", "r+") do |tty|
  tty.sync = true
  thread = Thread.new {
    while $keep_running do
      msg = burner.read.to_s
      next if msg.empty?
      if msg =~ /^[ -~]+$/
        tty.puts(">> #{msg}")
      else
        tty.puts(">> #{msg.inspect}")
      end
    end
  }

  burner.send_sync('C')

  while $keep_running && (line = tty.gets) do
    line.sub!(/[\r\n]*$/, '')
    line.strip!
    line.rstrip!
    fields = line.split
    next if fields.empty?
    case fields.first.downcase
    when ''
      next
    when 'q', 'quit'
      $keep_running = false
    when 'read'
      if fields.count == 2
        burner.read_to_file(fields[1])
      elsif fields.count == 4
        addresses = fields[2, 3].collect do |field|
          [ field.to_i(0), field.sub(/^[$]/, '').to_i(16) ].max
        end
        burner.read_to_file(fields[1], first: addresses.first, last: addresses.last)
      else
        burner.log "! Usage: read <filename.hex> [start] [end]"
      end
    when 'burn', 'write'
      if fields.count == 2
        burner.burn_from_ihex_file(fields[1])
      else
        burner.log "! Usage: burn <filename.hex>"
      end
    else
      burner.send_sync(line)
    end
  end

  thread.kill
  exit 0
end
