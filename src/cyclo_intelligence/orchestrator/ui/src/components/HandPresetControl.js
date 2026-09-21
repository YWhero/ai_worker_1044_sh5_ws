import React, { useEffect, useState } from 'react';
import { createPortal } from 'react-dom';
import clsx from 'clsx';
import { MdOpenInFull } from 'react-icons/md';

import useHandPresetStatus from '../hooks/useHandPresetStatus';
import HandPresetDialog from './HandPresetDialog';

const STATUS_STALE_AFTER_MS = 3000;

export default function HandPresetControl({ enabled = true }) {
  const status = useHandPresetStatus(enabled);
  const [isModalOpen, setIsModalOpen] = useState(false);
  const [now, setNow] = useState(Date.now());

  useEffect(() => {
    if (!enabled) return undefined;
    const timer = setInterval(() => setNow(Date.now()), 1000);
    return () => clearInterval(timer);
  }, [enabled]);

  useEffect(() => {
    if (!isModalOpen) return undefined;
    const handleKeyDown = (event) => {
      if (event.key === 'Escape') setIsModalOpen(false);
    };
    window.addEventListener('keydown', handleKeyDown);
    return () => window.removeEventListener('keydown', handleKeyDown);
  }, [isModalOpen]);

  if (!enabled) return null;

  const isOnline = Boolean(status) && now - status.receivedAt <= STATUS_STALE_AFTER_MS;
  const modal = isModalOpen && createPortal(
    <HandPresetDialog
      status={status}
      isOnline={isOnline}
      onClose={() => setIsModalOpen(false)}
    />,
    document.body
  );

  return (
    <>
      <div className="h-[85%] min-w-[230px] max-w-[280px] flex-1 overflow-hidden rounded-2xl border border-gray-200 bg-white p-3 shadow-md dark:border-slate-700 dark:bg-slate-900">
        <div className="flex items-center gap-2">
          <div className="flex h-9 w-9 items-center justify-center rounded-lg bg-indigo-100 text-sm font-bold text-indigo-700 dark:bg-indigo-950 dark:text-indigo-300">
            P
          </div>
          <div>
            <div className="text-sm font-semibold text-gray-800 dark:text-slate-100">Hand preset</div>
            <div className="text-[10px] text-gray-400 dark:text-slate-500">
              {status?.presetIds?.length || 0} registered
            </div>
          </div>
          <div className="ml-auto flex items-center gap-1.5 text-[10px] text-gray-500 dark:text-slate-400">
            <span className={clsx('h-2 w-2 rounded-full', isOnline ? 'bg-emerald-500' : 'bg-gray-300')} />
            {isOnline ? 'Online' : 'Waiting'}
          </div>
        </div>

        <div className="mt-3 grid grid-cols-2 gap-2">
          <div className="rounded-lg border border-gray-100 bg-gray-50 px-3 py-2 dark:border-slate-700 dark:bg-slate-800">
            <div className="text-[9px] font-semibold uppercase tracking-wide text-gray-400 dark:text-slate-400">Left</div>
            <div className="font-mono text-lg font-bold text-gray-800 dark:text-slate-100">
              {status?.leftPresetId ?? '—'}
            </div>
          </div>
          <div className="rounded-lg border border-gray-100 bg-gray-50 px-3 py-2 dark:border-slate-700 dark:bg-slate-800">
            <div className="text-[9px] font-semibold uppercase tracking-wide text-gray-400 dark:text-slate-400">Right</div>
            <div className="font-mono text-lg font-bold text-gray-800 dark:text-slate-100">
              {status?.rightPresetId ?? '—'}
            </div>
          </div>
        </div>

        <button
          type="button"
          onClick={() => setIsModalOpen(true)}
          className="mt-3 flex h-10 w-full items-center justify-center gap-2 rounded-lg bg-indigo-600 text-xs font-semibold text-white shadow-sm transition-colors hover:bg-indigo-700 focus:outline-none focus:ring-2 focus:ring-indigo-300 dark:bg-indigo-500 dark:hover:bg-indigo-400 dark:focus:ring-indigo-500"
          aria-label="Open hand preset selector"
        >
          <MdOpenInFull size={16} />
          Open preset studio
        </button>
      </div>
      {modal}
    </>
  );
}
