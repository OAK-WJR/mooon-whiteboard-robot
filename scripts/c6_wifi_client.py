#!/usr/bin/env python3
"""Local Wi-Fi transport for the existing C6 protocol. No calibration bypass.

Construction and --status use GET only; they never arm, stop or reset a robot.
Motion is available to the explicitly authorized loaded-capture workflow.
"""
import argparse
import json
import re
import time
from urllib.parse import urlsplit
from urllib.request import Request,build_opener,HTTPRedirectHandler,HTTPPasswordMgrWithDefaultRealm,HTTPDigestAuthHandler,ProxyHandler
from pathlib import Path


class NoRedirect(HTTPRedirectHandler):
    def redirect_request(self,*args,**kwargs):
        raise RuntimeError('Unexpected redirect from robot')


class WifiClient:
    def __init__(self,url,logfile=None,*,device_key=None):
        parsed=urlsplit(url)
        if (parsed.scheme!='http' or not parsed.hostname or parsed.username or parsed.password or
                parsed.query or parsed.fragment or parsed.path not in ('','/')):
            raise ValueError('Use the robot local http://hostname-or-IP/ root URL')
        self.base=url.rstrip('/');self.logfile=logfile
        passwords=HTTPPasswordMgrWithDefaultRealm()
        if device_key:passwords.add_password('WhiteBoardBot',self.base,'wbb',device_key)
        self.opener=build_opener(ProxyHandler({}),NoRedirect(),HTTPDigestAuthHandler(passwords))
        self.latest=None;self.frames=[];self.last_status=0;self.session=None;self.id=0
        self.token=None;self.commanded=False
        self._status()

    def _request(self,path,payload=None):
        headers={'Cache-Control':'no-store'}
        if payload is not None:headers.update({'Content-Type':'application/json','X-Control-Token':self.token})
        request=Request(self.base+path,data=None if payload is None else json.dumps(payload).encode(),headers=headers)
        with self.opener.open(request,timeout=.20) as response:
            data=response.read(65537)
        if len(data)>65536:raise RuntimeError('Oversized robot response')
        return data

    def _status(self):
        frame=json.loads(self._request('/status'))
        if frame.get('type')!='status' or frame.get('firmware')!='wbb-c6-drive-1':
            raise RuntimeError('Firmware mismatch')
        if self.session is not None and frame['session']!=self.session:raise RuntimeError('Board rebooted')
        self.session=frame['session'];self.latest=frame;self.last_status=time.monotonic()
        if not self.frames or frame['ms']!=self.frames[-1]['ms']:self.frames.append(frame)
        if self.logfile:self.logfile.write(json.dumps({'host':self.last_status,'frame':frame})+'\n')

    def pump(self,seconds):
        end=time.monotonic()+seconds
        while True:
            self._status()
            remaining=end-time.monotonic()
            if remaining<=0:return
            time.sleep(min(.035,remaining))

    def request(self,op,**args):
        if op in ('gains','pwm','validate_hold','validate_robust','save_controller','save','info'):
            raise ValueError('USB-only operation is not available over Wi-Fi')
        if self.token is None:
            page=self._request('/').decode('utf-8')
            match=re.search(r"const token='([0-9A-Fa-f]{16})'",page)
            if not match:raise RuntimeError('No matching control page/token')
            self.token=match.group(1)
        self.id+=1;self.commanded=True
        ack=json.loads(self._request('/cmd',dict(id=self.id,op=op,**args)))
        if ack.get('session')!=self.session:raise RuntimeError('Board rebooted')
        if ack.get('id')!=self.id or not ack.get('ok'):raise RuntimeError(f'{op}: {ack.get("error","bad acknowledgement")}')
        if self.logfile:self.logfile.write(json.dumps({'host':time.monotonic(),'frame':ack})+'\n')
        return ack

    def close(self):
        # A read-only observer must not stop someone else's active controller.
        if self.commanded:self.request('stop')


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--url',required=True);parser.add_argument('--status',action='store_true',required=True)
    parser.add_argument('--key-file',type=Path,help='Private file containing the device key on the screen, not the network password')
    args=parser.parse_args();client=WifiClient(args.url,device_key=args.key_file.read_text().strip() if args.key_file else None)
    print(json.dumps(client.latest,ensure_ascii=False,indent=2))
