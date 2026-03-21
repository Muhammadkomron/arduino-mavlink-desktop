export namespace backend {
	
	export class TelemetryData {
	    counter: number;
	    temperature: number;
	    humidity: number;
	    pressure: number;
	    altitude: number;
	    voltage: number;
	    accelX: number;
	    accelY: number;
	    accelZ: number;
	    gyroX: number;
	    gyroY: number;
	    gyroZ: number;
	    roll: number;
	    pitch: number;
	    yaw: number;
	    gyroRate: number;
	    gpsLat: number;
	    gpsLon: number;
	    gpsAlt: number;
	    gpsSats: number;
	    gpsFix: number;
	    msgCount: number;
	    packetsLost: number;
	    timestamp: string;
	    connected: boolean;
	    mode: string;
	    state: string;
	
	    static createFrom(source: any = {}) {
	        return new TelemetryData(source);
	    }
	
	    constructor(source: any = {}) {
	        if ('string' === typeof source) source = JSON.parse(source);
	        this.counter = source["counter"];
	        this.temperature = source["temperature"];
	        this.humidity = source["humidity"];
	        this.pressure = source["pressure"];
	        this.altitude = source["altitude"];
	        this.voltage = source["voltage"];
	        this.accelX = source["accelX"];
	        this.accelY = source["accelY"];
	        this.accelZ = source["accelZ"];
	        this.gyroX = source["gyroX"];
	        this.gyroY = source["gyroY"];
	        this.gyroZ = source["gyroZ"];
	        this.roll = source["roll"];
	        this.pitch = source["pitch"];
	        this.yaw = source["yaw"];
	        this.gyroRate = source["gyroRate"];
	        this.gpsLat = source["gpsLat"];
	        this.gpsLon = source["gpsLon"];
	        this.gpsAlt = source["gpsAlt"];
	        this.gpsSats = source["gpsSats"];
	        this.gpsFix = source["gpsFix"];
	        this.msgCount = source["msgCount"];
	        this.packetsLost = source["packetsLost"];
	        this.timestamp = source["timestamp"];
	        this.connected = source["connected"];
	        this.mode = source["mode"];
	        this.state = source["state"];
	    }
	}

}

export namespace common {
	
	export class MessageAttitude {
	    TimeBootMs: number;
	    Roll: number;
	    Pitch: number;
	    Yaw: number;
	    Rollspeed: number;
	    Pitchspeed: number;
	    Yawspeed: number;
	
	    static createFrom(source: any = {}) {
	        return new MessageAttitude(source);
	    }
	
	    constructor(source: any = {}) {
	        if ('string' === typeof source) source = JSON.parse(source);
	        this.TimeBootMs = source["TimeBootMs"];
	        this.Roll = source["Roll"];
	        this.Pitch = source["Pitch"];
	        this.Yaw = source["Yaw"];
	        this.Rollspeed = source["Rollspeed"];
	        this.Pitchspeed = source["Pitchspeed"];
	        this.Yawspeed = source["Yawspeed"];
	    }
	}
	export class MessageGpsRawInt {
	    TimeUsec: number;
	    FixType: number;
	    Lat: number;
	    Lon: number;
	    Alt: number;
	    Eph: number;
	    Epv: number;
	    Vel: number;
	    Cog: number;
	    SatellitesVisible: number;
	    AltEllipsoid: number;
	    HAcc: number;
	    VAcc: number;
	    VelAcc: number;
	    HdgAcc: number;
	    Yaw: number;
	
	    static createFrom(source: any = {}) {
	        return new MessageGpsRawInt(source);
	    }
	
	    constructor(source: any = {}) {
	        if ('string' === typeof source) source = JSON.parse(source);
	        this.TimeUsec = source["TimeUsec"];
	        this.FixType = source["FixType"];
	        this.Lat = source["Lat"];
	        this.Lon = source["Lon"];
	        this.Alt = source["Alt"];
	        this.Eph = source["Eph"];
	        this.Epv = source["Epv"];
	        this.Vel = source["Vel"];
	        this.Cog = source["Cog"];
	        this.SatellitesVisible = source["SatellitesVisible"];
	        this.AltEllipsoid = source["AltEllipsoid"];
	        this.HAcc = source["HAcc"];
	        this.VAcc = source["VAcc"];
	        this.VelAcc = source["VelAcc"];
	        this.HdgAcc = source["HdgAcc"];
	        this.Yaw = source["Yaw"];
	    }
	}
	export class MessageScaledImu2 {
	    TimeBootMs: number;
	    Xacc: number;
	    Yacc: number;
	    Zacc: number;
	    Xgyro: number;
	    Ygyro: number;
	    Zgyro: number;
	    Xmag: number;
	    Ymag: number;
	    Zmag: number;
	    Temperature: number;
	
	    static createFrom(source: any = {}) {
	        return new MessageScaledImu2(source);
	    }
	
	    constructor(source: any = {}) {
	        if ('string' === typeof source) source = JSON.parse(source);
	        this.TimeBootMs = source["TimeBootMs"];
	        this.Xacc = source["Xacc"];
	        this.Yacc = source["Yacc"];
	        this.Zacc = source["Zacc"];
	        this.Xgyro = source["Xgyro"];
	        this.Ygyro = source["Ygyro"];
	        this.Zgyro = source["Zgyro"];
	        this.Xmag = source["Xmag"];
	        this.Ymag = source["Ymag"];
	        this.Zmag = source["Zmag"];
	        this.Temperature = source["Temperature"];
	    }
	}
	export class MessageScaledPressure {
	    TimeBootMs: number;
	    PressAbs: number;
	    PressDiff: number;
	    Temperature: number;
	    TemperaturePressDiff: number;
	
	    static createFrom(source: any = {}) {
	        return new MessageScaledPressure(source);
	    }
	
	    constructor(source: any = {}) {
	        if ('string' === typeof source) source = JSON.parse(source);
	        this.TimeBootMs = source["TimeBootMs"];
	        this.PressAbs = source["PressAbs"];
	        this.PressDiff = source["PressDiff"];
	        this.Temperature = source["Temperature"];
	        this.TemperaturePressDiff = source["TemperaturePressDiff"];
	    }
	}
	export class MessageSysStatus {
	    OnboardControlSensorsPresent: number;
	    OnboardControlSensorsEnabled: number;
	    OnboardControlSensorsHealth: number;
	    Load: number;
	    VoltageBattery: number;
	    CurrentBattery: number;
	    BatteryRemaining: number;
	    DropRateComm: number;
	    ErrorsComm: number;
	    ErrorsCount1: number;
	    ErrorsCount2: number;
	    ErrorsCount3: number;
	    ErrorsCount4: number;
	    OnboardControlSensorsPresentExtended: number;
	    OnboardControlSensorsEnabledExtended: number;
	    OnboardControlSensorsHealthExtended: number;
	
	    static createFrom(source: any = {}) {
	        return new MessageSysStatus(source);
	    }
	
	    constructor(source: any = {}) {
	        if ('string' === typeof source) source = JSON.parse(source);
	        this.OnboardControlSensorsPresent = source["OnboardControlSensorsPresent"];
	        this.OnboardControlSensorsEnabled = source["OnboardControlSensorsEnabled"];
	        this.OnboardControlSensorsHealth = source["OnboardControlSensorsHealth"];
	        this.Load = source["Load"];
	        this.VoltageBattery = source["VoltageBattery"];
	        this.CurrentBattery = source["CurrentBattery"];
	        this.BatteryRemaining = source["BatteryRemaining"];
	        this.DropRateComm = source["DropRateComm"];
	        this.ErrorsComm = source["ErrorsComm"];
	        this.ErrorsCount1 = source["ErrorsCount1"];
	        this.ErrorsCount2 = source["ErrorsCount2"];
	        this.ErrorsCount3 = source["ErrorsCount3"];
	        this.ErrorsCount4 = source["ErrorsCount4"];
	        this.OnboardControlSensorsPresentExtended = source["OnboardControlSensorsPresentExtended"];
	        this.OnboardControlSensorsEnabledExtended = source["OnboardControlSensorsEnabledExtended"];
	        this.OnboardControlSensorsHealthExtended = source["OnboardControlSensorsHealthExtended"];
	    }
	}
	export class MessageVfrHud {
	    Airspeed: number;
	    Groundspeed: number;
	    Heading: number;
	    Throttle: number;
	    Alt: number;
	    Climb: number;
	
	    static createFrom(source: any = {}) {
	        return new MessageVfrHud(source);
	    }
	
	    constructor(source: any = {}) {
	        if ('string' === typeof source) source = JSON.parse(source);
	        this.Airspeed = source["Airspeed"];
	        this.Groundspeed = source["Groundspeed"];
	        this.Heading = source["Heading"];
	        this.Throttle = source["Throttle"];
	        this.Alt = source["Alt"];
	        this.Climb = source["Climb"];
	    }
	}

}

export namespace time {
	
	export class Time {
	
	
	    static createFrom(source: any = {}) {
	        return new Time(source);
	    }
	
	    constructor(source: any = {}) {
	        if ('string' === typeof source) source = JSON.parse(source);
	
	    }
	}

}

