import math
import copy
NAMES=list("ABCDE")
EDGES=["AB","BC","CD","DE"]

def obj(properties):
    return {'type': 'object', 'properties': properties,
            'required': list(properties), 'additionalProperties': False}

def schema():
    number = {'type': 'number'}
    point = obj({'pitch_deg': number, 'hold_s': number})
    segment = obj({'duration_s': number, 'midpoint_pitch_deg': number,
                   'joint4_amplitude_deg': number, 'oscillation_cycles': {'type': 'integer'}})
    return obj({'status': {'type': 'string', 'enum': ['ready', 'needs_clarification', 'unsupported']},
                'message': {'type': 'string'},
                'assumptions': {'type': 'array', 'items': {'type': 'string'}},
                'points': obj({n: copy.deepcopy(point) for n in NAMES}),
                'segments': obj({n: copy.deepcopy(segment) for n in EDGES})})

def defaults(targets):
    pitch = math.degrees(targets['pitch_rad'])
    return {'status': 'ready', 'message': '', 'assumptions': [],
            'points': {n: {'pitch_deg': pitch, 'hold_s': 0.} for n in NAMES},
            'segments': {e: {'duration_s': 6., 'midpoint_pitch_deg': pitch,
                            'joint4_amplitude_deg': 0., 'oscillation_cycles': 0} for e in EDGES}}

def check_shape(value, shape, path='spec'):
    kind = shape['type']
    if kind == 'object':
        if not isinstance(value, dict) or set(value) != set(shape['properties']):
            raise ValueError('Invalid fields at '+path)
        for k,s in shape['properties'].items():
            check_shape(value[k],s,path+'.'+k)
    elif kind == 'array':
        if not isinstance(value,list):
            raise ValueError('Expected array: '+path)
        for i,x in enumerate(value):
            check_shape(x,shape['items'],path+f'[{i}]')
    elif kind == 'string':
        if not isinstance(value,str) or ('enum' in shape and value not in shape['enum']):
            raise ValueError('Invalid string: '+path)
    elif kind == 'number':
        if type(value) not in (int,float) or not math.isfinite(value):
            raise ValueError('Invalid number: '+path)
    elif kind == 'integer':
        if type(value) is not int:
            raise ValueError('Expected integer: '+path)

def validate_parameters(spec, targets):
    check_shape(spec,schema())
    for n,p in spec['points'].items():
        if not -90 <= p['pitch_deg'] <= 90 or not 0 <= p['hold_s'] <= 30:
            raise ValueError('Pitch/hold out of supported range: '+n)
    for e,s in spec['segments'].items():
        if not 1 <= s['duration_s'] <= 60 or not -90 <= s['midpoint_pitch_deg'] <= 90:
            raise ValueError('Duration/pitch out of supported range: '+e)
        amp, cycles = s['joint4_amplitude_deg'], s['oscillation_cycles']
        if not 0 <= amp <= 45 or not 0 <= cycles <= 5 or ((amp == 0) != (cycles == 0)):
            raise ValueError('Invalid amplitude/cycle combination: '+e)
