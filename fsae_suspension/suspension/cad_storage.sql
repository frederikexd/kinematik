-- Team CAD library bytes in Supabase Storage (run once in the SQL editor).
--
-- The project document keeps only a pointer (CADFile.storage_path); the file
-- itself lives in this private bucket under "<workspace_id>/<file id>-<name>".
-- Members of a workspace can read and write its folder, nobody else can.
-- Until this has been run, uploads fail and files stay embedded as before.

insert into storage.buckets (id, name, public)
values ('kinematik-cad', 'kinematik-cad', false)
on conflict (id) do nothing;

create or replace function public.cad_folder_member(object_name text)
returns boolean language sql stable security definer set search_path = public as $$
    select case when (storage.foldername(object_name))[1] ~ '^[0-9a-f-]{36}$'
                then public.is_workspace_member(((storage.foldername(object_name))[1])::uuid)
                else false end;
$$;

drop policy if exists "cad read by members" on storage.objects;
create policy "cad read by members" on storage.objects for select to authenticated
    using (bucket_id = 'kinematik-cad' and public.cad_folder_member(name));

drop policy if exists "cad insert by members" on storage.objects;
create policy "cad insert by members" on storage.objects for insert to authenticated
    with check (bucket_id = 'kinematik-cad' and public.cad_folder_member(name));

drop policy if exists "cad update by members" on storage.objects;
create policy "cad update by members" on storage.objects for update to authenticated
    using (bucket_id = 'kinematik-cad' and public.cad_folder_member(name))
    with check (bucket_id = 'kinematik-cad' and public.cad_folder_member(name));
